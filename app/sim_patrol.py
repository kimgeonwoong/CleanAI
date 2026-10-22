import json, math, os, random, time, urllib.request

BASE = os.environ.get("SIM_BASE", "http://localhost:8000")
ROUTE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "patrol_routes.json")
SPEED_M = float(os.environ.get("SIM_SPEED", "12"))
DETECT_PROB = SPEED_M / float(os.environ.get("SIM_EVERY_M", "300"))   # about 1 per 300 m
JITTER_M = 3.0
TICK = float(os.environ.get("SIM_TICK", "1.0"))
CATS = ["plastic_bottle", "can", "paper", "vinyl", "general_trash", "cigarette_butt"]
K = 111320.0
BATT0 = [95.0, 88.0, 92.0]


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
    dy = (b[0] - a[0]) * K
    dx = (b[1] - a[1]) * K * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


class Walker:
    def __init__(self, rid, points, patrol_m, battery):
        self.rid, self.pts, self.patrol_m, self.battery = rid, points, patrol_m, battery
        self.cum = [0.0]
        for i in range(1, len(points)):
            self.cum.append(self.cum[-1] + dist_m(points[i - 1], points[i]))
        self.total = self.cum[-1]
        self.s = 0.0
        self.found = 0

    def point_at(self, s):
        s = max(0.0, min(self.total, s))
        for i in range(1, len(self.cum)):
            if s <= self.cum[i]:
                seg = self.cum[i] - self.cum[i - 1]
                fr = (s - self.cum[i - 1]) / seg if seg > 0 else 0.0
                a, b = self.pts[i - 1], self.pts[i]
                return (a[0] + (b[0] - a[0]) * fr, a[1] + (b[1] - a[1]) * fr)
        return tuple(self.pts[-1])

    def status(self):
        if self.s < self.patrol_m:
            return "patrol"
        if self.s < self.total:
            return "returning"
        return "patrol_done"


with open(ROUTE_FILE, encoding="utf-8") as f:
    routes = json.load(f)
walkers = [Walker(rid, d["points"], d["patrol_m"], BATT0[i % len(BATT0)])
           for i, (rid, d) in enumerate(routes.items())]
print("robots:", ", ".join(f"{w.rid}(patrol {w.patrol_m:.0f} m + return)" for w in walkers))

while True:
    for w in walkers:
        if w.status() != "patrol_done":
            w.s = min(w.total, w.s + SPEED_M)
            w.battery = max(0.0, w.battery - 0.05)
        st = w.status()
        lat, lng = w.point_at(w.s)
        if st == "patrol" and random.random() < DETECT_PROB:
            dlat = random.uniform(-JITTER_M, JITTER_M) / K
            dlng = random.uniform(-JITTER_M, JITTER_M) / (K * math.cos(math.radians(lat)))
            cat = random.choice(CATS)
            res = call("POST", "/detections", {
                "robot_id": w.rid, "lat": lat + dlat, "lng": lng + dlng,
                "category": cat, "confidence": round(random.uniform(0.6, 0.95), 2)})
            if "_error" in res:
                print("detection failed:", res["_error"])
            else:
                w.found += 1
                print(f"[{w.rid}] detected {cat}")
        call("POST", "/robots/status", {
            "robot_id": w.rid, "lat": lat, "lng": lng,
            "status": st, "battery": round(w.battery, 1)})
    if all(w.status() == "patrol_done" for w in walkers):
        break
    time.sleep(TICK)

print("patrol complete:", {w.rid: w.found for w in walkers})