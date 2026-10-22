"""
CleanAI FastAPI 서버
엔드포인트: /health, /detections, /robots/status, /robots, /route, /dashboard
"""

import asyncio
import json
import os
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import asyncpg
import redis.asyncio as aioredis
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from vrp_solver import WayPoint, solve_ortools, solve_nearest_neighbor

# ── 설정 ─────────────────────────────────────────────
DATABASE_URL = os.environ["DATABASE_URL"]
REDIS_URL = os.getenv("REDIS_URL", "redis://redis:6379")

# ── WebSocket 연결 관리 ───────────────────────────────
class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        self.active.remove(ws)

    async def broadcast(self, data: dict):
        msg = json.dumps(data, ensure_ascii=False, default=str)
        dead = []
        for ws in self.active:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.active.remove(ws)

manager = ConnectionManager()

# ── 앱 초기화 ─────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.pool = await asyncpg.create_pool(DATABASE_URL, min_size=2, max_size=10)
    app.state.redis = aioredis.from_url(REDIS_URL, decode_responses=True)
    yield
    await app.state.pool.close()
    await app.state.redis.close()

app = FastAPI(title="CleanAI FMS", version="2.0.0", lifespan=lifespan)
app.state.manager = manager

from fms import router as fms_router  # noqa: E402
app.include_router(fms_router)

# ── Pydantic 모델 ─────────────────────────────────────
class Detection(BaseModel):
    robot_id: str
    lat: float
    lng: float
    category: str
    confidence: float
    image_base64: Optional[str] = None

class RobotStatus(BaseModel):
    robot_id: str
    lat: float
    lng: float
    status: str = "patrol"
    battery: Optional[float] = None

class RouteRequest(BaseModel):
    algorithm: str = "ortools"   # "ortools" | "nn"
    use_road: bool = True

# ── 헬스 체크 ─────────────────────────────────────────
@app.get("/health")
async def health():
    return {"status": "ok", "timestamp": datetime.utcnow().isoformat()}

# ── 탐지 등록 ─────────────────────────────────────────
@app.post("/detections", status_code=201)
async def post_detection(det: Detection):
    async with app.state.pool.acquire() as conn:
        row = await conn.fetchrow(
            """INSERT INTO detections (robot_id, lat, lng, category, confidence, collected)
               VALUES ($1,$2,$3,$4,$5,false) RETURNING id, detected_at""",
            det.robot_id, det.lat, det.lng, det.category, det.confidence,
        )
    payload = {
        "event": "new_detection",
        "id": row["id"],
        "robot_id": det.robot_id,
        "lat": det.lat,
        "lng": det.lng,
        "category": det.category,
        "confidence": det.confidence,
        "detected_at": row["detected_at"].isoformat(),
    }
    await manager.broadcast(payload)
    await app.state.redis.lpush("detections:new", json.dumps(payload))
    return {"id": row["id"], "status": "created"}

# ── 탐지 목록 ─────────────────────────────────────────
@app.get("/detections")
async def get_detections(collected: Optional[bool] = None, limit: int = 100):
    async with app.state.pool.acquire() as conn:
        if collected is None:
            rows = await conn.fetch(
                "SELECT * FROM detections ORDER BY detected_at DESC LIMIT $1", limit)
        else:
            rows = await conn.fetch(
                "SELECT * FROM detections WHERE collected=$1 ORDER BY detected_at DESC LIMIT $2",
                collected, limit)
    return [dict(r) for r in rows]

@app.get("/detections/pending")
async def get_pending():
    async with app.state.pool.acquire() as conn:
        rows = await conn.fetch(
            "SELECT * FROM detections WHERE collected=false ORDER BY detected_at")
    return [dict(r) for r in rows]

@app.patch("/detections/{det_id}/collect")
async def collect_detection(det_id: int):
    async with app.state.pool.acquire() as conn:
        row = await conn.fetchrow(
            "UPDATE detections SET collected=true, collected_at=NOW() WHERE id=$1 RETURNING *",
            det_id)
    if not row:
        raise HTTPException(404, "detection not found")
    await manager.broadcast({"event": "collected", "id": det_id})
    return dict(row)

# ── 로봇 상태 ─────────────────────────────────────────
@app.post("/robots/status")
async def post_robot_status(status: RobotStatus):
    async with app.state.pool.acquire() as conn:
        await conn.execute(
            """INSERT INTO robot_status (robot_id, lat, lng, status, battery, updated_at)
               VALUES ($1,$2,$3,$4,$5,NOW())
               ON CONFLICT (robot_id) DO UPDATE
               SET lat=$2, lng=$3, status=$4, battery=$5, updated_at=NOW()""",
            status.robot_id, status.lat, status.lng, status.status, status.battery,
        )
    payload = {
        "event": "robot_update",
        "robot_id": status.robot_id,
        "lat": status.lat,
        "lng": status.lng,
        "status": status.status,
        "battery": status.battery,
    }
    await manager.broadcast(payload)
    return {"status": "updated"}

@app.get("/robots")
async def get_robots():
    async with app.state.pool.acquire() as conn:
        rows = await conn.fetch("SELECT * FROM robot_status ORDER BY robot_id")
    return [dict(r) for r in rows]

# ── 경로 최적화 ───────────────────────────────────────
@app.post("/route")
async def compute_route(req: RouteRequest):
    async with app.state.pool.acquire() as conn:
        rows = await conn.fetch(
            "SELECT id, lat, lng, category, confidence FROM detections WHERE collected=false ORDER BY detected_at")

    if not rows:
        return {"waypoints": [], "total_distance": 0, "algorithm": req.algorithm,
                "road_paths": []}

    waypoints = [WayPoint(id=r["id"], lat=r["lat"], lng=r["lng"],
                          category=r["category"], confidence=r["confidence"])
                 for r in rows]

    # 군산대 디지털정보관 (출발지)
    depot = WayPoint(id=0, lat=35.945149, lng=126.682853,
                     category="depot")

    if req.algorithm == "ortools":
        result = solve_ortools(waypoints, depot, time_limit_sec=5,
                               use_kakao=req.use_road)
    else:
        result = solve_nearest_neighbor(waypoints, depot, use_kakao=req.use_road)

    return {
        "algorithm": result.algorithm,
        "total_distance": round(result.total_distance, 1),
        "elapsed_ms": round(result.elapsed_ms, 1),
        "use_road": result.use_road,
        "depot": depot.to_dict(),
        "waypoints": [
            {**wp.to_dict(), "order": i + 1}
            for i, wp in enumerate(result.route)
        ],
        "road_paths": result.road_paths,
    }

# ── WebSocket ─────────────────────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()  # keep-alive ping
    except WebSocketDisconnect:
        manager.disconnect(ws)

# ── 대시보드 ──────────────────────────────────────────
@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard():
    html_path = Path(__file__).parent / "dashboard.html"
    return HTMLResponse(html_path.read_text(encoding="utf-8"))
