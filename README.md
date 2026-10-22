# CleanAI FMS Server

캠퍼스 쓰레기 탐지 로봇(AIST)의 관제 서버. 로봇이 탐지한 쓰레기 위치를 받아 저장하고, 수거 경로를 최적화(OR-Tools VRP, 카카오 실도로 거리)하여 지도 대시보드에 보여준다. 다중 로봇 배정(FMS)과 순찰 시뮬레이션을 포함한다.

## 구성
- `app/main.py` FastAPI 서버 (REST + WebSocket `/ws` + 대시보드 `/dashboard`)
- `app/vrp_solver.py` OR-Tools VRP / Nearest Neighbor, 카카오 Directions 도로 거리
- `app/fms.py` 다중 로봇 배정 (거리 0.5 + 배터리 0.3 + 작업량 0.2, 배터리 20% 미만 제외)
- `app/dashboard.html` Leaflet 관제 대시보드
- `app/gen_patrol_route.py`, `app/sim_patrol.py` 순찰 시뮬레이션 (가상 로봇 `sim_01~03`)
- `sql/init.sql` DB 스키마, `docker-compose.yml` (API + PostgreSQL 15 + Redis 7)

## 실행
1. 환경변수 파일 준비: `.env.example`을 `.env`로 복사하고 값 입력
   - `KAKAO_API_KEY` 카카오 Directions API 키
   - `DB_PASSWORD` PostgreSQL 비밀번호
2. `docker compose up -d`
3. 대시보드: http://localhost:8000/dashboard , API 문서: http://localhost:8000/docs

## 순찰 시뮬레이션 (발표 시연용, 가상 로봇·더미 데이터)

    docker compose exec api python /app/gen_patrol_route.py campus
    docker compose exec db psql -U cleanai -d cleanai -c "DELETE FROM detections;"
    docker compose exec -e SIM_SPEED=40 api python /app/sim_patrol.py

- 캠퍼스 도로(OpenStreetMap)를 3구역으로 나눠 가상 로봇이 구역별로 순찰하며 더미 쓰레기를 탐지하고, 끝나면 출발지로 복귀한다. 이후 대시보드의 경로 최적화 버튼으로 수거 경로를 계산한다.
- 도로 데이터는 `app/osm_cache_campus.json`에 캐시되어 인터넷 없이도 동작한다.
- `sim_` 로봇은 시뮬레이션이며 화면에 "시뮬레이션" 표시가 나타난다. 실제 로봇은 `robot_01` 등 다른 ID를 쓴다.

## 주요 API
- `POST /detections` 탐지 저장, `GET /detections/pending` 미수거 조회, `PATCH /detections/{id}/collect` 수거 완료
- `POST /route` 경로 최적화, `POST /robots/status` 로봇 상태, `GET /robots`
- `POST /fms/assign`, `POST /fms/unassign`, `GET /fms/tasks`, `POST /fms/route/{robot_id}`

## 보안
`.env`(API 키, DB 비밀번호)는 저장소에 올리지 않는다.