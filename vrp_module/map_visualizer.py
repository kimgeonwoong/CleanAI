"""
CleanAI - 수거 경로 지도 시각화
Folium으로 탐지 위치 마커 + 최적 경로 Polyline 생성
"""

import folium
from typing import List, Optional
from vrp_solver import WayPoint, RouteResult, haversine

# 카테고리별 색상
CATEGORY_COLOR = {
    "plastic_bottle": "green",
    "can":            "blue",
    "paper":          "orange",
    "vinyl":          "pink",
    "general_trash":  "gray",
    "cigarette_butt": "darkred",
    "unknown":        "lightgray",
}
CATEGORY_KR = {
    "plastic_bottle": "페트병",
    "can":            "캔",
    "paper":          "종이",
    "vinyl":          "비닐",
    "general_trash":  "일반쓰레기",
    "cigarette_butt": "담배꽁초",
}


def create_route_map(result: RouteResult,
                     compare: Optional[RouteResult] = None,
                     output_path: str = "route_map.html") -> str:
    """
    경로 최적화 결과를 지도에 시각화
    Args:
        result  : 주 경로 (OR-Tools 등)
        compare : 비교 경로 (Nearest Neighbor 등, 점선으로 표시)
        output_path: 저장 경로
    """
    # 지도 중심 계산
    all_wps = [result.depot] + result.route
    center_lat = sum(w.lat for w in all_wps) / len(all_wps)
    center_lng = sum(w.lng for w in all_wps) / len(all_wps)

    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=16,
        tiles="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        attr="OpenStreetMap"
    )

    # ── 출발지 마커 ──────────────────────────────────────
    folium.Marker(
        [result.depot.lat, result.depot.lng],
        popup="🏁 출발지 / 복귀지점",
        icon=folium.Icon(color="red", icon="home", prefix="fa")
    ).add_to(m)

    # ── 탐지 위치 마커 ────────────────────────────────────
    for i, wp in enumerate(result.route):
        color = CATEGORY_COLOR.get(wp.category, "gray")
        kr    = CATEGORY_KR.get(wp.category, wp.category)
        folium.CircleMarker(
            [wp.lat, wp.lng],
            radius=10,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.8,
            popup=folium.Popup(
                f"<b>{i+1}번째 수거</b><br>"
                f"종류: {kr}<br>"
                f"신뢰도: {wp.confidence:.0%}<br>"
                f"ID: #{wp.id}",
                max_width=200
            ),
            tooltip=f"{i+1}. {kr}"
        ).add_to(m)
        # 순서 번호 라벨
        folium.Marker(
            [wp.lat, wp.lng],
            icon=folium.DivIcon(
                html=f'<div style="font-size:11px;font-weight:bold;'
                     f'color:white;background:{color};border-radius:50%;'
                     f'width:20px;height:20px;text-align:center;line-height:20px;">'
                     f'{i+1}</div>',
                icon_size=(20, 20), icon_anchor=(10, 10)
            )
        ).add_to(m)

    # ── 주 경로 Polyline (실선, 파랑) ─────────────────────
    route_coords = ([[result.depot.lat, result.depot.lng]] +
                    [[wp.lat, wp.lng] for wp in result.route] +
                    [[result.depot.lat, result.depot.lng]])
    folium.PolyLine(
        route_coords,
        color="#1565C0",
        weight=4,
        opacity=0.85,
        tooltip=f"{result.algorithm}: {result.total_distance:.0f}m"
    ).add_to(m)

    # ── 비교 경로 Polyline (점선, 빨강) ───────────────────
    if compare:
        compare_coords = ([[compare.depot.lat, compare.depot.lng]] +
                          [[wp.lat, wp.lng] for wp in compare.route] +
                          [[compare.depot.lat, compare.depot.lng]])
        folium.PolyLine(
            compare_coords,
            color="#C62828",
            weight=3,
            opacity=0.5,
            dash_array="8 4",
            tooltip=f"{compare.algorithm}: {compare.total_distance:.0f}m"
        ).add_to(m)

    # ── 범례 ─────────────────────────────────────────────
    legend_html = f"""
    <div style="position:fixed;bottom:30px;left:30px;z-index:1000;
                background:white;padding:12px;border-radius:8px;
                box-shadow:2px 2px 8px rgba(0,0,0,0.3);font-size:13px;">
        <b>CleanAI 수거 경로</b><br>
        <span style="color:#1565C0">━━</span> {result.algorithm} ({result.total_distance:.0f}m)<br>
        {'<span style="color:#C62828">╌╌</span> ' + compare.algorithm + f' ({compare.total_distance:.0f}m)<br>' if compare else ''}
        <hr style="margin:4px 0">
        🟢 페트병 &nbsp; 🔵 캔 &nbsp; 🟠 종이<br>
        🩷 비닐 &nbsp; ⚫ 일반 &nbsp; 🔴 담배꽁초<br>
        🏁 출발/복귀 지점
    </div>"""
    m.get_root().html.add_child(folium.Element(legend_html))

    m.save(output_path)
    print(f"지도 저장 완료: {output_path}")
    return output_path
