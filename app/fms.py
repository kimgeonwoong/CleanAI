import math
from fastapi import APIRouter, HTTPException, Request
from vrp_solver import WayPoint, solve_ortools, solve_nearest_neighbor

router = APIRouter(prefix="/fms", tags=["fms"])

# assignment priority: distance > battery > load (weights sum to 1.0)
W_DIST, W_BATT, W_LOAD = 0.5, 0.3, 0.2
MIN_BATTERY = 20.0


def haversine_km(lat1, lng1, lat2, lng2):
    r = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lng2 - lng1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


@router.post("/assign")
async def assign(request: Request):
    pool = request.app.state.pool
    async with pool.acquire() as conn:
        robots = await conn.fetch(
            "SELECT robot_id, lat, lng, battery FROM robot_status "
            "WHERE lat IS NOT NULL AND status <> 'offline'")
        dets = await conn.fetch(
            "SELECT id, lat, lng FROM detections "
            "WHERE collected=false AND assigned_robot IS NULL ORDER BY detected_at")
        rows = await conn.fetch(
            "SELECT assigned_robot, COUNT(*) AS n FROM detections "
            "WHERE collected=false AND assigned_robot IS NOT NULL GROUP BY assigned_robot")
    loads = {r["assigned_robot"]: r["n"] for r in rows}
    robots = [r for r in robots if r["battery"] is None or r["battery"] >= MIN_BATTERY]
    if not robots:
        raise HTTPException(409, "no available robot")

    result = []
    for d in dets:
        dists = {r["robot_id"]: haversine_km(r["lat"], r["lng"], d["lat"], d["lng"]) for r in robots}
        max_d = max(dists.values()) or 1.0
        best, best_score = None, None
        for r in robots:
            rid = r["robot_id"]
            batt = r["battery"] if r["battery"] is not None else 100.0
            score = (W_DIST * dists[rid] / max_d
                     + W_BATT * (100.0 - batt) / 100.0
                     + W_LOAD * min(loads.get(rid, 0) / 5.0, 1.0))
            if best_score is None or score < best_score:
                best, best_score = rid, score
        loads[best] = loads.get(best, 0) + 1
        result.append({"detection_id": d["id"], "robot_id": best, "score": round(best_score, 3)})

    if result:
        async with pool.acquire() as conn:
            await conn.executemany(
                "UPDATE detections SET assigned_robot=$2 WHERE id=$1",
                [(x["detection_id"], x["robot_id"]) for x in result])
        await request.app.state.manager.broadcast({"event": "assigned", "assignments": result})
    return {"assigned": result, "loads": loads}


@router.post("/unassign")
async def unassign(request: Request):
    async with request.app.state.pool.acquire() as conn:
        await conn.execute("UPDATE detections SET assigned_robot=NULL WHERE collected=false")
    await request.app.state.manager.broadcast({"event": "unassigned"})
    return {"status": "cleared"}


@router.get("/tasks")
async def tasks(request: Request):
    async with request.app.state.pool.acquire() as conn:
        rows = await conn.fetch(
            "SELECT id, lat, lng, category, assigned_robot FROM detections "
            "WHERE collected=false AND assigned_robot IS NOT NULL ORDER BY assigned_robot, detected_at")
    out = {}
    for r in rows:
        out.setdefault(r["assigned_robot"], []).append(
            {"id": r["id"], "lat": r["lat"], "lng": r["lng"], "category": r["category"]})
    return out


@router.post("/route/{robot_id}")
async def robot_route(robot_id: str, request: Request, algorithm: str = "nn", use_road: bool = False):
    async with request.app.state.pool.acquire() as conn:
        robot = await conn.fetchrow("SELECT lat, lng FROM robot_status WHERE robot_id=$1", robot_id)
        rows = await conn.fetch(
            "SELECT id, lat, lng, category, confidence FROM detections "
            "WHERE collected=false AND assigned_robot=$1 ORDER BY detected_at", robot_id)
    if not robot:
        raise HTTPException(404, "robot not found")
    if not rows:
        return {"robot_id": robot_id, "waypoints": [], "total_distance": 0}
    wps = [WayPoint(id=r["id"], lat=r["lat"], lng=r["lng"],
                    category=r["category"], confidence=r["confidence"]) for r in rows]
    depot = WayPoint(id=0, lat=robot["lat"], lng=robot["lng"], category="depot")
    if algorithm == "ortools":
        res = solve_ortools(wps, depot, time_limit_sec=5, use_kakao=use_road)
    else:
        res = solve_nearest_neighbor(wps, depot, use_kakao=use_road)
    return {
        "robot_id": robot_id,
        "algorithm": res.algorithm,
        "total_distance": round(res.total_distance, 1),
        "use_road": res.use_road,
        "depot": depot.to_dict(),
        "waypoints": [{**wp.to_dict(), "order": i + 1} for i, wp in enumerate(res.route)],
        "road_paths": res.road_paths,
    }