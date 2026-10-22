"""
CleanAI - 수거 경로 최적화 모듈
OR-Tools VRP + Nearest Neighbor 휴리스틱 비교
Kakao Directions API 기반 실제 도로 거리 사용
"""

import math
import time
import requests
from dataclasses import dataclass
from typing import List, Dict, Optional

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


# ── Kakao Directions API ─────────────────────────────
import os
KAKAO_API_KEY = os.environ.get("KAKAO_API_KEY", "")


def road_distance(lat1: float, lng1: float,
                  lat2: float, lng2: float) -> float:
    """
    카카오 Directions API로 실제 도로 거리 반환 (미터)
    실패 시 직선거리(Haversine)로 자동 fallback
    """
    url = "https://apis-navi.kakaomobility.com/v1/directions"
    headers = {"Authorization": f"KakaoAK {KAKAO_API_KEY}"}
    params = {
        "origin":      f"{lng1},{lat1}",
        "destination": f"{lng2},{lat2}",
        "priority":    "DISTANCE",
    }
    try:
        resp = requests.get(url, headers=headers, params=params, timeout=3)
        data = resp.json()
        return float(data["routes"][0]["summary"]["distance"])
    except Exception:
        return haversine(lat1, lng1, lat2, lng2)


# ── 데이터 타입 ──────────────────────────────────────
@dataclass
class WayPoint:
    id: int
    lat: float
    lng: float
    category: str = "unknown"
    confidence: float = 1.0

    def __repr__(self):
        return f"WP#{self.id}({self.category[:3]}@{self.lat:.5f},{self.lng:.5f})"


@dataclass
class RouteResult:
    algorithm: str
    route: List[WayPoint]
    total_distance: float
    elapsed_ms: float
    depot: WayPoint
    use_road: bool = False

    def print_summary(self):
        dist_type = "도로 거리" if self.use_road else "직선 거리"
        print(f"\n{'='*50}")
        print(f"알고리즘  : {self.algorithm}")
        print(f"거리 기준 : {dist_type}")
        print(f"총 거리   : {self.total_distance:.1f}m ({self.total_distance/1000:.2f}km)")
        print(f"연산 시간 : {self.elapsed_ms:.1f}ms")
        print(f"수거 순서 ({len(self.route)}곳):")
        prev = self.depot
        for i, wp in enumerate(self.route):
            d = haversine(prev.lat, prev.lng, wp.lat, wp.lng)
            print(f"  {i+1}. {wp} ({d:.0f}m 이동)")
            prev = wp
        d_back = haversine(prev.lat, prev.lng, self.depot.lat, self.depot.lng)
        print(f"  → 출발지 복귀 ({d_back:.0f}m)")
        print(f"{'='*50}")


# ── 거리 계산 ─────────────────────────────────────────
def haversine(lat1: float, lng1: float,
              lat2: float, lng2: float) -> float:
    """두 GPS 좌표 간 직선 거리 (미터)"""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lng2 - lng1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def build_distance_matrix(points: List[WayPoint],
                           use_kakao: bool = True) -> List[List[int]]:
    """거리 행렬 생성 (OR-Tools용 정수 행렬)"""
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
                          use_kakao: bool = True) -> float:
    """경로 총 거리 계산 (카카오 도로 거리 기준)"""
    if not route:
        return 0.0

    def get_dist(a_lat, a_lng, b_lat, b_lng):
        if use_kakao:
            return road_distance(a_lat, a_lng, b_lat, b_lng)
        return haversine(a_lat, a_lng, b_lat, b_lng)

    total = get_dist(depot.lat, depot.lng, route[0].lat, route[0].lng)
    for i in range(len(route)-1):
        total += get_dist(route[i].lat, route[i].lng,
                          route[i+1].lat, route[i+1].lng)
    total += get_dist(route[-1].lat, route[-1].lng, depot.lat, depot.lng)
    return total


# ── 알고리즘 1: OR-Tools VRP (정확해) ─────────────────
def solve_ortools(waypoints: List[WayPoint], depot: WayPoint,
                  time_limit_sec: int = 5,
                  use_kakao: bool = True) -> RouteResult:
    """OR-Tools로 최적 수거 경로 계산"""
    t0 = time.time()
    all_points = [depot] + waypoints

    if use_kakao:
        print(f"  카카오 API로 거리 행렬 계산 중 ({len(all_points)}x{len(all_points)})...")
    matrix = build_distance_matrix(all_points, use_kakao=use_kakao)

    manager = pywrapcp.RoutingIndexManager(len(matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def dist_cb(from_idx, to_idx):
        return matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]

    transit_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    params.time_limit.seconds = time_limit_sec

    solution = routing.SolveWithParameters(params)

    if not solution:
        return RouteResult("OR-Tools (실패)", [], 0,
                           (time.time()-t0)*1000, depot, use_kakao)

    route = []
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        node = manager.IndexToNode(idx)
        if node != 0:
            route.append(waypoints[node-1])
        idx = solution.Value(routing.NextVar(idx))

    elapsed = (time.time() - t0) * 1000
    # 실제 총 거리는 카카오 기준으로 재계산
    dist = total_route_distance(route, depot, use_kakao=use_kakao)
    return RouteResult("OR-Tools VRP", route, dist, elapsed, depot, use_kakao)


# ── 알고리즘 2: Nearest Neighbor (빠른 근사해) ──────────
def solve_nearest_neighbor(waypoints: List[WayPoint],
                            depot: WayPoint,
                            use_kakao: bool = False) -> RouteResult:
    """
    가장 가까운 미방문 지점을 순서대로 방문
    실시간 동적 재계획용
    """
    t0 = time.time()
    unvisited = list(waypoints)
    route = []
    current_lat, current_lng = depot.lat, depot.lng

    while unvisited:
        if use_kakao:
            nearest = min(unvisited,
                          key=lambda wp: road_distance(
                              current_lat, current_lng, wp.lat, wp.lng))
        else:
            nearest = min(unvisited,
                          key=lambda wp: haversine(
                              current_lat, current_lng, wp.lat, wp.lng))
        route.append(nearest)
        current_lat, current_lng = nearest.lat, nearest.lng
        unvisited.remove(nearest)

    elapsed = (time.time() - t0) * 1000
    # 총 거리 카카오 기준으로 계산
    dist = total_route_distance(route, depot, use_kakao=use_kakao)
    return RouteResult("Nearest Neighbor", route, dist, elapsed, depot, use_kakao)


# ── 동적 재계획 ──────────────────────────────────────
def replan(current_lat: float, current_lng: float,
           remaining: List[WayPoint]) -> RouteResult:
    """수거 진행 중 신규 탐지 발생 시 현재 위치 기준 즉시 재계획"""
    current = WayPoint(id=0, lat=current_lat, lng=current_lng,
                       category="current_position")
    return solve_nearest_neighbor(remaining, current, use_kakao=False)


# ── 알고리즘 비교 ────────────────────────────────────
def compare_algorithms(waypoints: List[WayPoint],
                        depot: WayPoint,
                        use_kakao: bool = True) -> Dict:
    """두 알고리즘 성능 비교 (동일 기준으로 공정하게)"""
    dist_type = "카카오 도로 거리" if use_kakao else "직선 거리"
    print(f"\n경로 최적화 비교 ({len(waypoints)}개 수거 지점, {dist_type})")
    print(f"출발지: ({depot.lat:.5f}, {depot.lng:.5f})\n")

    # NN은 직선거리로 빠르게 경로 계산 후 카카오로 총 거리 재계산
    nn = solve_nearest_neighbor(waypoints, depot, use_kakao=False)
    if use_kakao:
        print("  NN 경로 카카오 거리 재계산 중...")
        nn.total_distance = total_route_distance(nn.route, depot, use_kakao=True)
        nn.use_road = True

    # OR-Tools는 카카오 거리로 최적화
    ort = solve_ortools(waypoints, depot, use_kakao=use_kakao)

    nn.print_summary()
    ort.print_summary()

    if nn.total_distance > 0:
        improvement = (nn.total_distance - ort.total_distance) / nn.total_distance * 100
    else:
        improvement = 0.0

    print(f"\n📊 비교 결과 (동일 기준: {dist_type})")
    print(f"  Nearest Neighbor : {nn.total_distance:.0f}m  ({nn.elapsed_ms:.1f}ms)")
    print(f"  OR-Tools VRP     : {ort.total_distance:.0f}m  ({ort.elapsed_ms:.0f}ms)")
    print(f"  OR-Tools 개선율  : {improvement:.1f}%")

    return {"nearest_neighbor": nn, "ortools": ort,
            "improvement_pct": improvement}