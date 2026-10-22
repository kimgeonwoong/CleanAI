"""
CleanAI - 수거 경로 최적화 모듈
OR-Tools VRP + Nearest Neighbor 비교
Kakao Directions API 기반 실제 도로 거리 사용
"""

import math
import time
import requests
from typing import List, Optional

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ── Kakao Directions API ──────────────────────────────
import os
KAKAO_API_KEY = os.environ.get("KAKAO_API_KEY", "")


def road_distance(lat1: float, lng1: float,
                  lat2: float, lng2: float) -> float:
    """카카오 Directions API로 실제 도로 거리(m), 실패 시 직선 fallback"""
    url = "https://apis-navi.kakaomobility.com/v1/directions"
    headers = {"Authorization": f"KakaoAK {KAKAO_API_KEY}"}
    params = {
        "origin": f"{lng1},{lat1}",
        "destination": f"{lng2},{lat2}",
        "priority": "DISTANCE",
    }
    try:
        resp = requests.get(url, headers=headers, params=params, timeout=3)
        data = resp.json()
        return float(data["routes"][0]["summary"]["distance"])
    except Exception:
        return haversine(lat1, lng1, lat2, lng2)


def get_road_path(lat1: float, lng1: float,
                  lat2: float, lng2: float) -> list:
    """카카오 API에서 실제 도로 경로 좌표 목록 반환 [[lat,lng], ...]"""
    url = "https://apis-navi.kakaomobility.com/v1/directions"
    headers = {"Authorization": f"KakaoAK {KAKAO_API_KEY}"}
    params = {
        "origin": f"{lng1},{lat1}",
        "destination": f"{lng2},{lat2}",
        "priority": "DISTANCE",
    }
    try:
        resp = requests.get(url, headers=headers, params=params, timeout=5)
        data = resp.json()
        coords = []
        for section in data["routes"][0]["sections"]:
            for road in section["roads"]:
                verts = road["vertexes"]
                for i in range(0, len(verts), 2):
                    coords.append([verts[i + 1], verts[i]])  # [lat, lng]
        return coords
    except Exception:
        return [[lat1, lng1], [lat2, lng2]]


# ── 데이터 타입 ───────────────────────────────────────
class WayPoint:
    def __init__(self, id: int, lat: float, lng: float,
                 category: str = "unknown", confidence: float = 1.0):
        self.id = id
        self.lat = lat
        self.lng = lng
        self.category = category
        self.confidence = confidence

    def to_dict(self):
        return {
            "id": self.id,
            "lat": self.lat,
            "lng": self.lng,
            "category": self.category,
            "confidence": self.confidence,
        }

    def __repr__(self):
        return f"WP#{self.id}({self.category[:4]}@{self.lat:.5f},{self.lng:.5f})"


class RouteResult:
    def __init__(self, algorithm: str, route: List[WayPoint],
                 total_distance: float, elapsed_ms: float,
                 depot: WayPoint, use_road: bool = False,
                 road_paths: Optional[list] = None):
        self.algorithm = algorithm
        self.route = route
        self.total_distance = total_distance
        self.elapsed_ms = elapsed_ms
        self.depot = depot
        self.use_road = use_road
        self.road_paths = road_paths or []  # 실제 도로 경로 좌표


# ── 거리 계산 ─────────────────────────────────────────
def haversine(lat1: float, lng1: float,
              lat2: float, lng2: float) -> float:
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lng2 - lng1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def build_distance_matrix(points: List[WayPoint],
                          use_kakao: bool = True) -> List[List[int]]:
    n = len(points)
    mat = []
    for i in range(n):
        row = []
        for j in range(n):
            if i == j:
                row.append(0)
                continue
            if use_kakao:
                d = road_distance(points[i].lat, points[i].lng,
                                  points[j].lat, points[j].lng)
            else:
                d = haversine(points[i].lat, points[i].lng,
                              points[j].lat, points[j].lng)
            row.append(int(d))
        mat.append(row)
    return mat


def total_route_distance(route: List[WayPoint], depot: WayPoint,
                         use_kakao: bool = False) -> float:
    if not route:
        return 0.0

    def get_d(a, b):
        if use_kakao:
            return road_distance(a.lat, a.lng, b.lat, b.lng)
        return haversine(a.lat, a.lng, b.lat, b.lng)

    total = get_d(depot, route[0])
    for i in range(len(route) - 1):
        total += get_d(route[i], route[i + 1])
    total += get_d(route[-1], depot)
    return total


def build_road_paths(route: List[WayPoint], depot: WayPoint) -> list:
    """경로 전체의 실제 도로 좌표 세그먼트 목록"""
    segments = []
    points = [depot] + route + [depot]
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        path = get_road_path(a.lat, a.lng, b.lat, b.lng)
        segments.append(path)
    return segments


# ── OR-Tools VRP ──────────────────────────────────────
def solve_ortools(waypoints: List[WayPoint], depot: WayPoint,
                  time_limit_sec: int = 5,
                  use_kakao: bool = True) -> RouteResult:
    t0 = time.time()

    all_points = [depot] + waypoints
    matrix = build_distance_matrix(all_points, use_kakao=use_kakao)

    manager = pywrapcp.RoutingIndexManager(len(matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_cb(from_idx, to_idx):
        return matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]

    transit_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = time_limit_sec

    solution = routing.SolveWithParameters(params)

    route = []
    if solution:
        idx = routing.Start(0)
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                route.append(waypoints[node - 1])
            idx = solution.Value(routing.NextVar(idx))

    elapsed = (time.time() - t0) * 1000

    # 카카오 도로 거리로 재계산
    dist = total_route_distance(route, depot, use_kakao=use_kakao)
    road_paths = build_road_paths(route, depot) if route else []

    return RouteResult("OR-Tools VRP", route, dist, elapsed, depot, use_kakao, road_paths)


# ── Nearest Neighbor ──────────────────────────────────
def solve_nearest_neighbor(waypoints: List[WayPoint],
                           depot: WayPoint,
                           use_kakao: bool = False) -> RouteResult:
    t0 = time.time()
    unvisited = list(waypoints)
    route = []
    cur_lat, cur_lng = depot.lat, depot.lng

    while unvisited:
        if use_kakao:
            nearest = min(unvisited, key=lambda wp: road_distance(cur_lat, cur_lng, wp.lat, wp.lng))
        else:
            nearest = min(unvisited, key=lambda wp: haversine(cur_lat, cur_lng, wp.lat, wp.lng))
        route.append(nearest)
        cur_lat, cur_lng = nearest.lat, nearest.lng
        unvisited.remove(nearest)

    elapsed = (time.time() - t0) * 1000
    dist = total_route_distance(route, depot, use_kakao=use_kakao)
    road_paths = build_road_paths(route, depot) if route else []

    return RouteResult("Nearest Neighbor", route, dist, elapsed, depot, use_kakao, road_paths)


# ── 동적 재계획 ───────────────────────────────────────
def replan(current_lat: float, current_lng: float,
           remaining: List[WayPoint]) -> RouteResult:
    current = WayPoint(id=0, lat=current_lat, lng=current_lng,
                       category="current_position")
    return solve_nearest_neighbor(remaining, current, use_kakao=False)