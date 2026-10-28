#!/usr/bin/env bash
# M1 실행 (단계 6-2): ROS-TCP 엔드포인트 + 관절 상태 소스를 함께 띄우고 Ctrl-C 로 함께 끈다.
#
#   bash scripts/run_m1.sh fake [fake_joint_states.py 인자…]   가짜 퍼블리셔 (Isaac 없이)
#   bash scripts/run_m1.sh bag  [bag 경로, 기본 ~/bags/run1] [ros2 bag play 인자…]   Isaac 런 bag 을 반복 재생
#        인자를 안 주면 --start-offset 44 로 앞쪽 대기 구간을 건너뛴다(~/bags/run1 은 처음 46.7 s 동안 팔이 overview 에 서 있다 —
#        Isaac 녹화를 켠 뒤 노드 기동·트리거를 기다린 시간. 첫 잡기는 69 s). 처음부터 보려면 --start-offset 0
#        M1_BAG_ONCE=1 이면 반복 없이 한 번만(시험용: 처음으로 되감기는 점프가 속도 위반으로 잡히지 않게)
#        예) M1_BAG_ONCE=1 bash scripts/run_m1.sh bag ~/bags/run1 --start-offset 40 --topics /dsr01/joint_states
#
# 그 다음 수동 확인(H3): bash scripts/open_editor.sh mirror (에디터·씬·Play 자동). docs/HUMAN_STEPS.md H3-1.
# M1 에는 딸기 노드가 없어 Isaac HUD 패널이 없다 — 시작할 때 이전 M2 런이 남긴 HUD 뷰 파일을 지운다(남아 있으면 '수확 완료' 가 뜬다).
#
# - 도메인·RMW 는 run_endpoint.sh 와 같은 기본값(0, rmw_fastrtps_cpp = CONTRACT §7).
# - 로그: logs/m1/<STAMP>/{endpoint,source}.log  (딸기 리포 밖, D-5)
# - 엔드포인트는 run_endpoint.sh 가 exec 하므로 그 PID 에 SIGTERM 을 주면 바로 죽는다(단계 2 기록).
# - Ctrl-C(또는 TERM)는 정리하고 끝낸다(종료 코드 130/143). 정리 중 Ctrl-C 는 무시한다.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
MODE="${1:-}"; shift || true
ENDPOINT_PORT="${ENDPOINT_PORT:-10000}"

case "$MODE" in
    fake|bag) ;;
    *) echo "사용법: $0 fake [퍼블리셔 인자…] | bag [bag 경로]" >&2; exit 2 ;;
esac

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONDONTWRITEBYTECODE=1

STAMP="$(date +%Y%m%d_%H%M%S)"
# M1_LOG_DIR 로 바꿀 수 있다(unity_test_m1.sh 가 자기 결과 폴더를 넘겨 한 실행 = 한 폴더가 되게 한다)
LOG_DIR="${M1_LOG_DIR:-$SIM_ROOT/logs/m1/$STAMP}"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
    # 정리는 끝까지 한다: 여기서부터 Ctrl-C·TERM 은 무시한다(run_m2_headless.sh 와 같은 방식).
    trap '' INT TERM
    trap - EXIT
    echo
    echo "[run_m1] 종료 중… (정리 중 Ctrl-C 는 무시한다)"
    for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    # 5초 안에 안 죽으면 KILL. (python 은 SIGTERM 에 ExternalShutdownException 으로 정상 종료한다)
    for _ in $(seq 1 20); do
        alive=0; for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ "$alive" = 0 ] && break; sleep 0.25
    done
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && { echo "[run_m1] $p 가 안 끝나 KILL"; kill -KILL "$p" 2>/dev/null; }; done
    for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null || true; done
    echo "[run_m1] 로그: ${LOG_DIR#"$SIM_ROOT"/}/"
}
on_signal() {  # Ctrl-C·TERM: 정리하고 그 자리에서 끝낸다
    cleanup
    exit "$1"
}
# 다른 스크립트가 이 스크립트를 백그라운드로 띄우면 SIGINT 는 무시(SIG_IGN) 상태로 시작해 trap 이 걸리지 않는다(bash 규칙).
# 그래서 스크립트에서 끌 때는 SIGTERM 을 보내야 한다. 터미널의 Ctrl-C(INT)는 포그라운드라 정상 동작한다.
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

echo "[run_m1] 모드=$MODE  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  로그=${LOG_DIR#"$SIM_ROOT"/}"

# 1) 엔드포인트
bash "$SIM_ROOT/scripts/run_endpoint.sh" > "$LOG_DIR/endpoint.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 1 50); do
    ss -ltn 2>/dev/null | grep -qE "[:.]${ENDPOINT_PORT}\b" && break
    kill -0 "${PIDS[0]}" 2>/dev/null || { echo "!! 엔드포인트가 바로 죽었다:" >&2; cat "$LOG_DIR/endpoint.log" >&2; exit 1; }
    sleep 0.2
done
if ! ss -ltn 2>/dev/null | grep -qE "[:.]${ENDPOINT_PORT}\b"; then
    echo "!! 엔드포인트가 10초 안에 포트 ${ENDPOINT_PORT} 를 열지 않았다:" >&2; cat "$LOG_DIR/endpoint.log" >&2; exit 1
fi
echo "[run_m1] 엔드포인트 127.0.0.1:${ENDPOINT_PORT} 준비 (pid ${PIDS[0]})"

# 이전 M2 런의 HUD 뷰 파일(scripts/hud_relay.py 출력)을 지운다 — M1 에는 HUD 데이터가 없다
rm -f "${E0509_HUD_VIEW:-/tmp/e0509_hud_view.json}"

# 2) 관절 상태 소스
if [ "$MODE" = fake ]; then
    # `… | tee f &` 로 띄우면 $! 가 tee 라서 종료 때 퍼블리셔가 살아남는다. 프로세스 치환으로 $! 를 python 에 둔다.
    python3 -B "$SIM_ROOT/scripts/fake_joint_states.py" "$@" > >(tee "$LOG_DIR/source.log") 2>&1 &
    PIDS+=($!)
    echo "[run_m1] 가짜 퍼블리셔 시작: fake_joint_states.py $*"
else
    BAG="${1:-$HOME/bags/run1}"
    if [ ! -d "$BAG" ]; then
        echo "!! bag 폴더 없음: $BAG (H4 녹화 먼저, CONTRACT §7-5)" >&2; exit 1
    fi
    # -l 반복 재생. 녹화된 토픽 중 /dsr01/joint_states 만 미러에 쓰이고 나머지는 M1 에서 무해하다.
    shift || true
    [ $# -eq 0 ] && set -- --start-offset 44     # 앞쪽 대기 구간 건너뛰기(위 설명)
    LOOP=(-l); [ "${M1_BAG_ONCE:-0}" = "1" ] && LOOP=()
    ros2 bag play "${LOOP[@]}" "$BAG" "$@" > >(tee "$LOG_DIR/source.log") 2>&1 &
    PIDS+=($!)
    echo "[run_m1] bag 재생$([ ${#LOOP[@]} -eq 0 ] && echo '(한 번)' || echo '(반복)'): $BAG $*"
fi

echo "[run_m1] Ctrl-C 로 전부 종료. 에디터: bash scripts/open_editor.sh mirror (H3-1)."
wait "${PIDS[1]}"
