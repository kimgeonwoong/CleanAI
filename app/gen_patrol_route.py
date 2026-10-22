import heapq, json, math, os, sys, time, urllib.parse, urllib.request

DEPOT = (35.945149, 126.682853)
ARG = sys.argv[1] if len(sys.argv) > 1 else "500"
CAMPUS = ARG == "campus"
RADIUS = 500 if CAMPUS else int(ARG)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "patrol_routes.json")
ROBOT_IDS = ["sim_01", "sim_02", "sim_03"]
SERVERS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]
K = 111320.0
QUERY = (
    '[out:json][timeout:40];'
    'way(around:{r},{lat},{lng})'
    '["highway"~"^(footway|pedestrian|path|service|residential|living_street|unclassified|tertiary|cycleway)$"]'
    '["area"!="yes"]["access"!~"^(private|no)$"]["service"!~"^(parking_aisle|driveway)$"];'
    'out geom;'
)


CAMPUS_QUERY = (
    '[out:json][timeout:60];'
    '(rel["amenity"="university"]["name"~"\uad70\uc0b0\ub300"];way["amenity"="university"]["name"~"\uad70\uc0b0\ub300"];'
    'rel["amenity"="university"]["name:en"~"Kunsan"];way["amenity"="university"]["name:en"~"Kunsan"];)->.u;'
    '.u map_to_area->.a;'
    'way(area.a)["highway"~"^(footway|pedestrian|service|residential|living_street|unclassified|tertiary|cycleway)$"]'
    '["area"!="yes"]["access"!~"^(private|no)$"]["service"!~"^(parking_aisle|driveway)$"];'
    'out geom;'
)


def dist_m(a, b):
    dy = (b[0] - a[0]) * K
    dx = (b[1] - a[1]) * K * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def fetch_ways(radius):
    q = CAMPUS_QUERY if CAMPUS else QUERY.format(r=radius, lat=DEPOT[0], lng=DEPOT[1])
    body = urllib.parse.urlencode({"data": q}).encode()
    last = None
    for url in SERVERS:
        for _ in range(4):
            try:
                req = urllib.request.Request(url, data=body, headers={"User-Agent": "cleanai-fms/1.0"})
                with urllib.request.urlopen(req, timeout=60) as r:
                    return json.loads(r.read().decode())["elements"]
            except Exception as e:
                last = e
                print("overpass failed:", url, e)
                time.sleep(5)
    raise SystemExit(f"could not fetch road data: {last}")


def build_graph(elements):
    coords, adj = {}, {}
    for w in elements:
        if w.get("type") != "way" or "nodes" not in w or "geometry" not in w:
            continue
        ids = w["nodes"]
        for nid, g in zip(ids, w["geometry"]):
            coords[nid] = (g["lat"], g["lon"])
        for a, b in zip(ids, ids[1:]):
            if a == b or a not in coords or b not in coords:
                continue
            d = dist_m(coords[a], coords[b])
            adj.setdefault(a, {})[b] = d
            adj.setdefault(b, {})[a] = d
    return coords, adj


def largest_component(adj):
    seen, best = set(), []
    for s in adj:
        if s in seen:
            continue
        comp, stack = [], [s]
        seen.add(s)
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in adj[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        if len(comp) > len(best):
            best = comp
    return set(best)


def split_zones(coords, edges, k):
    es = list(edges)
    mids = {e: ((coords[e[0]][0] + coords[e[1]][0]) / 2, (coords[e[0]][1] + coords[e[1]][1]) / 2) for e in es}
    seeds = [max(es, key=lambda e: dist_m(mids[e], DEPOT))]
    while len(seeds) < k:
        seeds.append(max(es, key=lambda e: min(dist_m(mids[e], mids[s]) for s in seeds)))
    centers = [mids[s] for s in seeds]
    groups = [[] for _ in range(k)]
    for _ in range(20):
        groups = [[] for _ in range(k)]
        for e in es:
            i = min(range(k), key=lambda j: dist_m(mids[e], centers[j]))
            groups[i].append(e)
        newc = []
        for j in range(k):
            g = groups[j]
            if g:
                newc.append((sum(mids[e][0] for e in g) / len(g), sum(mids[e][1] for e in g) / len(g)))
            else:
                newc.append(centers[j])
        if newc == centers:
            break
        centers = newc
    return groups


def dijkstra_to(adj, cur, goal):
    dist, prev, pq = {cur: 0.0}, {}, [(0.0, cur)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u != cur and goal(u):
            p = [u]
            while p[-1] != cur:
                p.append(prev[p[-1]])
            return p[::-1]
        for v, w in adj[u].items():
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return None


def zone_tour(adj, nodes, req_edges, start):
    unvisited = set(req_edges)
    open_cnt = {n: 0 for n in nodes}
    for a, b in unvisited:
        open_cnt[a] += 1
        open_cnt[b] += 1
    tour, last_req = [start], 0

    def mark(a, b):
        e = (a, b) if a < b else (b, a)
        if e in unvisited:
            unvisited.discard(e)
            open_cnt[a] -= 1
            open_cnt[b] -= 1
            return True
        return False

    cur = start
    while unvisited:
        nxt = [v for v in adj[cur] if ((cur, v) if cur < v else (v, cur)) in unvisited]
        if nxt:
            v = min(nxt, key=lambda x: adj[cur][x])
            mark(cur, v)
            tour.append(v)
            last_req = len(tour) - 1
            cur = v
            continue
        p = dijkstra_to(adj, cur, lambda u: open_cnt[u] > 0)
        if not p:
            break
        for a, b in zip(p, p[1:]):
            removed = mark(a, b)
            tour.append(b)
            if removed:
                last_req = len(tour) - 1
        cur = p[-1]
    patrol_end = len(tour)
    if cur != start:
        back = dijkstra_to(adj, cur, lambda u: u == start)
        if back:
            tour.extend(back[1:])
    return tour, patrol_end


def balance(coords, adj, comp, groups, start, iters=800):
    k = len(groups)
    groups = [list(g) for g in groups]

    def mid(e):
        return ((coords[e[0]][0] + coords[e[1]][0]) / 2, (coords[e[0]][1] + coords[e[1]][1]) / 2)

    def plen(g):
        if not g:
            return 0.0
        tour, pe = zone_tour(adj, comp, g, start)
        t = tour[:pe]
        return sum(adj[a][b] for a, b in zip(t, t[1:]))

    lens = [plen(g) for g in groups]
    for _ in range(iters):
        hi = max(range(k), key=lambda i: lens[i])
        lo = min(range(k), key=lambda i: lens[i])
        if lens[hi] - lens[lo] <= 0.12 * lens[hi]:
            break
        lo_nodes = {n for e in groups[lo] for n in e}
        if groups[lo]:
            cl = (sum(mid(e)[0] for e in groups[lo]) / len(groups[lo]),
                  sum(mid(e)[1] for e in groups[lo]) / len(groups[lo]))
        else:
            cl = DEPOT
        cands = sorted(groups[hi], key=lambda e: (0 if (e[0] in lo_nodes or e[1] in lo_nodes) else 1,
                                                  dist_m(mid(e), cl)))[:30]
        moved = False
        for e in cands:
            g_hi = [x for x in groups[hi] if x != e]
            g_lo = groups[lo] + [e]
            nh, nl = plen(g_hi), plen(g_lo)
            if max(nh, nl) < lens[hi] - 1.0:
                groups[hi], groups[lo] = g_hi, g_lo
                lens[hi], lens[lo] = nh, nl
                moved = True
                break
        if not moved:
            break
    return groups


def main():
    cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"osm_cache_{'campus' if CAMPUS else RADIUS}.json")
    if os.path.exists(cache):
        with open(cache, encoding="utf-8") as f:
            els = json.load(f)
        print("using cache:", cache)
    else:
        els = fetch_ways(RADIUS)
        if not els:
            raise SystemExit("no roads found (empty result)")
        with open(cache, "w", encoding="utf-8") as f:
            json.dump(els, f)
    print("ways:", len(els))
    coords, adj = build_graph(els)
    if not adj:
        raise SystemExit("no roads found")
    comp = largest_component(adj)
    print("road nodes:", len(comp))
    edges = {(a, b) for a in comp for b in adj[a] if a < b}
    start = min(comp, key=lambda n: dist_m(coords[n], DEPOT))
    groups = split_zones(coords, edges, len(ROBOT_IDS))
    groups = balance(coords, adj, comp, groups, start)
    result = {}
    for rid, g in zip(ROBOT_IDS, groups):
        tour, patrol_end = zone_tour(adj, comp, g, start)
        pts = [[round(coords[n][0], 6), round(coords[n][1], 6)] for n in tour]
        cum = 0.0
        patrol_m = 0.0
        for i in range(1, len(pts)):
            cum += dist_m(pts[i - 1], pts[i])
            if i == patrol_end - 1:
                patrol_m = cum
        if patrol_end <= 1:
            patrol_m = 0.0
        zone_m = sum(adj[a][b] for a, b in g)
        result[rid] = {"points": pts, "patrol_m": round(patrol_m, 1)}
        print(f"{rid}: zone roads {zone_m:.0f} m, patrol {patrol_m:.0f} m, total with return {cum:.0f} m")
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(result, f)
    print("saved", OUT)


if __name__ == "__main__":
    main()