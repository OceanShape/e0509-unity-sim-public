#!/usr/bin/env bash
# ROS-TCP-Endpoint 를 띄운다 (단계 2). Unity(ROS-TCP-Connector) 가 여기로 붙는다.
#
#   bash scripts/run_endpoint.sh                     127.0.0.1:10000, 도메인 0
#   ENDPOINT_PORT=10001 bash scripts/run_endpoint.sh  포트만 바꿈
#   ROS_DOMAIN_ID=77 bash scripts/run_endpoint.sh     시험용 도메인 (기본은 CONTRACT §7 의 0)
#
# 딸기 워크스페이스는 소싱만 한다(읽기 전용). 엔드포인트가 dsr_msgs2 같은 커스텀 타입을
# import 할 수 있게 하려는 것이다. 경계 토픽은 전부 표준 타입이라 M1·M2 에는 꼭 필요하지 않다.
# import 할 때 딸기 install/ 아래에 __pycache__ 가 생기지 않게 PYTHONDONTWRITEBYTECODE=1 을 준다.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
STRAWBERRY_WS="${STRAWBERRY_WS:-$HOME/strawberry_grasp_environment}"
ENDPOINT_IP="${ENDPOINT_IP:-127.0.0.1}"
ENDPOINT_PORT="${ENDPOINT_PORT:-10000}"

if [ ! -f "$SIM_ROOT/ros/install/setup.bash" ]; then
    echo "!! $SIM_ROOT/ros/install/setup.bash 없음 — 먼저 빌드: cd ros && colcon build" >&2
    exit 1
fi
if ss -ltn 2>/dev/null | grep -qE "[:.]${ENDPOINT_PORT}\b"; then
    echo "!! 포트 ${ENDPOINT_PORT} 를 이미 누가 쓰고 있다:" >&2
    ss -ltnp 2>/dev/null | grep -E "[:.]${ENDPOINT_PORT}\b" >&2
    exit 1
fi

# ROS setup.bash 는 미정의 변수를 읽으므로 이 구간만 set -u 를 끈다.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$STRAWBERRY_WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$STRAWBERRY_WS/install/setup.bash"
else
    echo "[run_endpoint] 경고: 딸기 ws 없음($STRAWBERRY_WS) — 커스텀 타입은 못 쓴다" >&2
fi
# shellcheck disable=SC1091
source "$SIM_ROOT/ros/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"              # CONTRACT §7: 딸기 쪽은 미설정 = 0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"   # Isaac 쪽과 같게 명시
export PYTHONDONTWRITEBYTECODE=1

echo "[run_endpoint] ${ENDPOINT_IP}:${ENDPOINT_PORT}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
# `ros2 run` 래퍼를 거치지 않고 실행 파일을 exec 한다. 래퍼에 SIGTERM 을 보내면 래퍼만 죽고
# 엔드포인트가 고아로 남아 포트를 계속 잡는다(10-20 확인). exec 하면 이 셸의 PID 가 곧 엔드포인트다.
EP_BIN="$(ros2 pkg prefix ros_tcp_endpoint)/lib/ros_tcp_endpoint/default_server_endpoint"
exec "$EP_BIN" --ros-args -p ROS_IP:="${ENDPOINT_IP}" -p ROS_TCP_PORT:="${ENDPOINT_PORT}"
