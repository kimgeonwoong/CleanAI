"""
CleanAI - 수거 경로 계획기 메인
FastAPI 서버에서 미수거 탐지 데이터를 가져와 경로 최적화 실행
출발지: 군산대학교 디지털정보관
"""

import argparse
import json
import sys
import requests
from typing import List, Optional

from vrp_solver import (WayPoint, compare_algorithms,
                        solve_ortools, solve_nearest_neighbor,
                        replan, total_route_distance)
from map_visualizer import create_route_map


# ── 군산대 캠퍼스 샘플 데이터 ─────────────────────────
SAMPLE_WAYPOINTS = [
    WayPoint(id=1, lat=35.944781, lng=126.679939,
             category="plastic_bottle", confidence=0.92),   # 중앙도서관
    WayPoint(id=2, lat=35.942532, lng=126.683079,
             category="can",            confidence=0.87),   # 아카데미홀
    WayPoint(id=3, lat=35.946354, lng=126.681717,
             category="paper",          confidence=0.75),   # 종합교육관
    WayPoint(id=4, lat=35.948659, lng=126.680927,
             category="vinyl",          confidence=0.83),   # 사회대 운동장
    WayPoint(id=5, lat=35.943500, lng=126.682500,
             category="cigarette_butt", confidence=0.91),   # 캠퍼스 내
]

# 출발지: 디지털정보관
SAMPLE_DEPOT = WayPoint(
    id=0, lat=35.945149, lng=126.682853, category="depot"
)

API_SERVER = "http://localhost:8000"


# ── 서버에서 데이터 가져오기 ──────────────────────────
def fetch_from_server(server_url: str) -> Optional[List[WayPoint]]:
    """FastAPI 서버에서 미수거 탐지 위치 조회"""
    try:
        resp = requests.get(f"{server_url}/detections/pending", timeout=5)
        resp.raise_for_status()
        data = resp.json()
        if not data:
            print("미수거 탐지 데이터 없음")
            return []
        wps = [WayPoint(
            id=d["id"],
            lat=float(d["latitude"]),
            lng=float(d["longitude"]),
            category=d["category"],
            confidence=float(d["confidence"])
        ) for d in data]
        print(f"서버에서 {len(wps)}개 미수거 위치 로드")
        return wps
    except requests.ConnectionError:
        print(f"서버 연결 실패 ({server_url}) → 샘플 데이터 사용")
        return None
    except Exception as e:
        print(f"서버 오류: {e} → 샘플 데이터 사용")
        return None


def main(args):
    print("\n🗺  CleanAI 수거 경로 최적화 시작")
    print(f"  출발지: 디지털정보관 ({SAMPLE_DEPOT.lat}, {SAMPLE_DEPOT.lng})")

    # ── 데이터 로드 ─────────────────────────────────
    if args.sample:
        print("샘플 데이터 사용 (--sample 플래그)")
        waypoints = SAMPLE_WAYPOINTS
        depot     = SAMPLE_DEPOT
    else:
        waypoints = fetch_from_server(args.server)
        depot     = WayPoint(
            id=0,
            lat=args.depot_lat,
            lng=args.depot_lng,
            category="depot"
        )
        if waypoints is None:
            print("샘플 데이터로 대체합니다")
            waypoints = SAMPLE_WAYPOINTS
            depot     = SAMPLE_DEPOT

    if not waypoints:
        print("수거할 위치가 없습니다.")
        sys.exit(0)

    # ── 경로 최적화 실행 ────────────────────────────
    if args.compare:
        results = compare_algorithms(
            waypoints, depot, use_kakao=not args.no_kakao)
        best    = results["ortools"]
        compare = results["nearest_neighbor"]
    else:
        if args.algorithm == "nn":
            best    = solve_nearest_neighbor(
                waypoints, depot, use_kakao=not args.no_kakao)
            compare = None
        else:
            best    = solve_ortools(
                waypoints, depot,
                time_limit_sec=args.time_limit,
                use_kakao=not args.no_kakao)
            compare = None
        best.print_summary()

    # ── 동적 재계획 데모 ────────────────────────────
    if args.replan_demo and len(best.route) > 2:
        print("\n🔄 동적 재계획 시뮬레이션")
        print("  가정: 1번 수거 완료 후 신규 탐지 발생")
        current_wp = best.route[0]
        remaining  = best.route[1:]
        new_wp = WayPoint(
            id=99,
            lat=current_wp.lat + 0.0003,
            lng=current_wp.lng + 0.0003,
            category="plastic_bottle",
            confidence=0.95
        )
        remaining.append(new_wp)
        replan_result = replan(current_wp.lat, current_wp.lng, remaining)
        replan_result.print_summary()

    # ── 지도 생성 ───────────────────────────────────
    if not args.no_map:
        out = args.output if args.output else "route_map.html"
        create_route_map(best, compare if args.compare else None, out)
        print(f"\n브라우저에서 열기: {out}")

    # ── 경로 JSON 출력 ─────────────────────────────
    route_json = {
        "algorithm":        best.algorithm,
        "total_distance_m": round(best.total_distance, 1),
        "elapsed_ms":       round(best.elapsed_ms, 1),
        "depot": {
            "name": "디지털정보관",
            "lat": depot.lat,
            "lng": depot.lng
        },
        "route": [
            {
                "order":      i + 1,
                "id":         wp.id,
                "lat":        wp.lat,
                "lng":        wp.lng,
                "category":   wp.category,
                "confidence": wp.confidence,
            }
            for i, wp in enumerate(best.route)
        ]
    }
    print(f"\n경로 JSON:\n{json.dumps(route_json, ensure_ascii=False, indent=2)}")
    return route_json


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="CleanAI 수거 경로 최적화")
    p.add_argument("--server",      default=API_SERVER)
    p.add_argument("--depot-lat",   type=float, default=35.945149,
                   help="출발지 위도 (기본: 디지털정보관)")
    p.add_argument("--depot-lng",   type=float, default=126.682853,
                   help="출발지 경도 (기본: 디지털정보관)")
    p.add_argument("--algorithm",   choices=["ortools", "nn"], default="ortools")
    p.add_argument("--time-limit",  type=int,   default=5,
                   help="OR-Tools 탐색 시간 제한 (초)")
    p.add_argument("--compare",     action="store_true",
                   help="OR-Tools vs Nearest Neighbor 비교")
    p.add_argument("--replan-demo", action="store_true",
                   help="동적 재계획 시뮬레이션")
    p.add_argument("--sample",      action="store_true",
                   help="샘플 데이터 사용 (서버 불필요)")
    p.add_argument("--no-kakao",    action="store_true",
                   help="카카오 API 비활성화 (직선거리 사용)")
    p.add_argument("--no-map",      action="store_true")
    p.add_argument("--output",      default="route_map.html")
    main(p.parse_args())