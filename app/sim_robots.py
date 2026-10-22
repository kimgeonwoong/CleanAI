import json, math, random, time, urllib.request
CATS = ["plastic_bottle", "can", "paper", "vinyl", "general_trash", "cigarette_butt"]

BASE = "http://localhost:8000"
DEPOT = (35.945149, 126.682853)
STEP_M = 8.0          # meters per tick (1 tick = 1 s)
TICK = 1.0
ASSIGN_EVERY = 5      # ticks


def call(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            txt = r.read().decode()
            return json.loads(txt) if txt else {}
    except Exception as e:
        return {"_error": str(e)}


def dist_m(a, b):
    k = 111320.0
    dy = (b[0] - a[0]) * k
    dx = (b[1] - a[1]) * k * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def step_toward(pos, tgt, step):
    d = dist_m(pos, tgt)
    if d <= step:
        return tgt, True
    f = step / d
    return (pos[0] + (tgt[0] - pos[0]) * f, pos[1] + (tgt[1] - pos[1]) * f), False


robots = {
    "sim_01": {"pos": (DEPOT[0], DEPOT[1] + 0.0002), "battery": 95.0},
    "sim_02": {"pos": (DEPOT[0] + 0.0003, DEPOT[1]), "battery": 70.0},
    "sim_03": {"pos": (DEPOT[0] - 0.0003, DEPOT[1] - 0.0002), "battery": 40.0},
}

print("simulation robots:", ", ".join(robots), "(Ctrl+C to stop)")
tick = 0
while True:
    tick += 1
    if tick % ASSIGN_EVERY == 1:
        call("POST", "/fms/assign")
    tasks = call("GET", "/fms/tasks")
    if "_error" in tasks:
        tasks = {}
    for rid, r in robots.items():
        mine = tasks.get(rid, [])
        if r["battery"] < 20:
            target, status = DEPOT, "charging"
        elif mine:
            t = min(mine, key=lambda x: dist_m(r["pos"], (x["lat"], x["lng"])))
            target, status = (t["lat"], t["lng"]), "moving"
        else:
            target, status = DEPOT, "idle"
        r["pos"], arrived = step_toward(r["pos"], target, STEP_M if status != "idle" or r["pos"] != DEPOT else 0)
        if status == "moving":
            r["battery"] = max(0.0, r["battery"] - 0.15)
            if arrived:
                res = call("PATCH", f"/detections/{t['id']}/collect")
                if "_error" in res:
                    print(f"[{rid}] collect failed: {res['_error']}")
                    continue
                print(f"[{rid}] collected #{t['id']} ({t['category']})")
        elif status == "charging":
            r["battery"] = min(100.0, r["battery"] + 1.0)
        call("POST", "/robots/status", {
            "robot_id": rid, "lat": r["pos"][0], "lng": r["pos"][1],
            "status": status, "battery": round(r["battery"], 1)})
    if tick % 12 == 0:
        pending = call("GET", "/detections?collected=false&limit=200")
        n = len(pending) if isinstance(pending, list) else 99
        if n < 8:
            res = call("POST", "/detections", {
                "robot_id": random.choice(list(robots)),
                "lat": DEPOT[0] + random.uniform(-0.0015, 0.0015),
                "lng": DEPOT[1] + random.uniform(-0.0020, 0.0020),
                "category": random.choice(CATS),
                "confidence": round(random.uniform(0.6, 0.95), 2)})
            if "_error" in res:
                print("dummy detection failed:", res["_error"])
    time.sleep(TICK)