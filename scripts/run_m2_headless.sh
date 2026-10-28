#!/usr/bin/env bash
# M2 헤드리스 런 (단계 7): Isaac 없이 플래너 → 가상 제어기 → Unity(Command 모드) → 상태 회신 루프를 한 번 돈다.
#
#   bash scripts/run_m2_headless.sh [--isaac-speeds] [--max-min 25] [--plant batch|editor]
#     --plant editor : Unity 를 띄우지 않는다. 에디터에서 E0509Sim 을 Command 모드로 Play 해 둔 것을 플랜트로 쓴다
#                      (H3 Command 확인·영상 녹화용). 상태가 overview 로 올 때까지 최대 5분 기다린다.
#
# 순서 (CONTRACT §5-4 기동 순서 계약: 플랜트 먼저):
#   1. ROS-TCP 엔드포인트
#   2. Unity batchmode PlayMode 테스트 M2PlantRun (Command 모드, ROS 연결) — 플랜트
#   3. /dsr01/joint_states 가 overview ±1° 로 나오는지 확인 (check_overview.py)
#   4. ROS 쪽 주기 측정 시작 (ros_rate_monitor.py)
#   5. 딸기 노드 4개 (run_strawberry_nodes.sh, 기동 로그 대조 전부 정합이어야 진행)
#   6. overview 재확인 → 트리거 /strawberry/scan/start
#   7. scan.log 에 SCAN_COMPLETE / START_REJECTED / EXEC_TIMEOUT 또는 시간 상한까지 대기
#   8. 정리(노드 → 주기 측정 → Unity 정지 파일 → 엔드포인트) 후 m2_summary.py
# Ctrl-C(또는 TERM)는 그 자리에서 정리하고 끝낸다(종료 코드 130/143, 결과 interrupted). 정리 중 Ctrl-C 는 무시한다.
# 결과: logs/m2/<STAMP>_m2_headless/ (unity.log 는 git 제외). 실패는 다시 시도하지 않고 보고한다.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
PROJECT="$SIM_ROOT/UnityProject"
NODE_ARGS=(); MAX_MIN=25; PLANT=batch
while [ $# -gt 0 ]; do
    case "$1" in
        --isaac-speeds) NODE_ARGS+=(--isaac-speeds); shift ;;
        --max-min) MAX_MIN="$2"; shift 2 ;;
        --plant) PLANT="$2"; shift 2; [ "$PLANT" = batch ] || [ "$PLANT" = editor ] || { echo "!! --plant batch|editor" >&2; exit 2; } ;;
        *) echo "!! 모르는 인자: $1" >&2; exit 2 ;;
    esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$SIM_ROOT/logs/m2/${STAMP}_m2_$([ "$PLANT" = editor ] && echo editor || echo headless)"
mkdir -p "$OUT"
STOP_FILE="$OUT/stop_plant"
touch "$OUT/.run_start"     # 이 런 동안 생긴 플래너 JSONL 을 고르는 기준 시각
log() { echo "[run_m2] $(date +%H:%M:%S) $*" | tee -a "$OUT/orchestrator.log"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONDONTWRITEBYTECODE=1

# ── 사전 점검 ──
if [ "$PLANT" = batch ]; then
for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        echo "!! Unity 가 이미 $PROJECT 를 열고 있다 (pid $pid). 에디터를 닫은 뒤 다시 실행 (에디터를 플랜트로 쓰려면 --plant editor)." >&2; exit 1
    fi
done
fi
if pgrep -af "[d]efault_server_endpoint|[f]ake_joint_states.py|[f]ake_vision_node|[s]im_executor_bridge_node|[c]urobo_planner_node|[s]can_executor_node" >/dev/null; then
    echo "!! 엔드포인트·가짜 퍼블리셔·딸기 노드가 이미 떠 있다. 먼저 정리:" >&2
    pgrep -af "[d]efault_server_endpoint|[f]ake_joint_states.py|[f]ake_vision_node|[s]im_executor_bridge_node|[c]urobo_planner_node|[s]can_executor_node" >&2
    exit 1
fi
[ "$PLANT" = batch ] && [ -f "$PROJECT/Temp/UnityLockfile" ] && { log "남은 Temp/UnityLockfile 삭제 (Unity 프로세스 없음 확인함)"; rm -f "$PROJECT/Temp/UnityLockfile"; }

# 프로세스를 자기 세션/그룹 리더로 띄우고 PID 를 파일로 받는다(setsid 가 fork 해도 PID 가 맞다).
spawn() {  # spawn <pidfile> <logfile> <cmd…>
    local pidf="$1" logf="$2"; shift 2
    setsid bash -c 'echo $$ > "$0"; exec "$@"' "$pidf" "$@" > "$logf" 2>&1 < /dev/null &
    for _ in $(seq 1 50); do [ -s "$pidf" ] && break; sleep 0.1; done
}
stop_group() {  # stop_group <pidfile> <이름> <대기초>
    local pidf="$1" name="$2" wait_s="$3" pid
    [ -s "$pidf" ] || return 0
    pid="$(cat "$pidf")"
    kill -0 "$pid" 2>/dev/null || return 0
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    for _ in $(seq 1 $((wait_s * 4))); do kill -0 "$pid" 2>/dev/null || break; sleep 0.25; done
    if kill -0 "$pid" 2>/dev/null; then log "$name 이 ${wait_s}s 안에 안 끝나 KILL"; kill -KILL -- "-$pid" 2>/dev/null; fi
}

RESULT="unknown"
cleanup() {
    # 정리는 끝까지 한다: 여기서부터 Ctrl-C·TERM 은 무시한다(이후 띄우는 자식도 물려받는다).
    # 전에는 trap 을 풀어 정리 중 두 번째 Ctrl-C 에 스크립트가 죽고 엔드포인트·주기 측정이 남았다(20261022_003352).
    trap '' INT TERM
    trap - EXIT
    log "정리 시작 (결과: $RESULT) — 정리 중 Ctrl-C 는 무시한다, 끝날 때까지 기다린다 (꼭 멈추려면 Ctrl-\\)"
    stop_group "$OUT/nodes.pid" "딸기 노드" 30
    stop_group "$OUT/ratemon.pid" "주기 측정" 10
    touch "$STOP_FILE"
    if [ -s "$OUT/unity.pid" ]; then
        UP="$(cat "$OUT/unity.pid")"
        for _ in $(seq 1 240); do kill -0 "$UP" 2>/dev/null || break; sleep 0.25; done
        kill -0 "$UP" 2>/dev/null && { log "Unity 가 60 s 안에 안 끝나 TERM"; stop_group "$OUT/unity.pid" "Unity" 20; }
    fi
    stop_group "$OUT/endpoint.pid" "엔드포인트" 10
    # 에디터 플랜트면 모니터 CSV 를 복사해 둔다(에디터가 Play 중이어도 읽을 수 있다)
    if [ "$PLANT" = editor ]; then
        cp "$SIM_ROOT/logs/limit_violations.csv" "$SIM_ROOT/logs/limit_near.csv" "$OUT/" 2>/dev/null
    fi
    # Unity 가 강제로 끝났으면 잠금 파일이 남는다. Unity 프로세스가 없을 때만 지운다.
    if [ "$PLANT" = batch ] && ! pgrep -f "^${UNITY_BIN}( |$)" >/dev/null && [ -f "$PROJECT/Temp/UnityLockfile" ]; then rm -f "$PROJECT/Temp/UnityLockfile"; log "Temp/UnityLockfile 삭제"; fi
    # 이 런의 플래너 JSONL 사본
    if [ -d "$SIM_ROOT/logs/runtime" ]; then
        mkdir -p "$OUT/runtime"
        find "$SIM_ROOT/logs/runtime" -type f -newer "$OUT/.run_start" -exec cp {} "$OUT/runtime/" \; 2>/dev/null
    fi
    left="$(pgrep -af "[d]efault_server_endpoint|[f]ake_vision_node|[s]im_executor_bridge_node|[c]urobo_planner_node|[s]can_executor_node|[r]os_rate_monitor")"
    [ -n "$left" ] && { log "!! 남은 프로세스:"; echo "$left" | tee -a "$OUT/orchestrator.log"; }
    python3 -B "$SIM_ROOT/scripts/m2_summary.py" "$OUT" 2>&1 | tee -a "$OUT/orchestrator.log"
    log "결과 폴더: ${OUT#"$SIM_ROOT"/}"
}
on_signal() {  # Ctrl-C·TERM: 정리하고 그 자리에서 끝낸다(정리 뒤 본 흐름으로 돌아가 대기를 이어 가지 않게)
    [ "$RESULT" = "unknown" ] && RESULT="interrupted"
    cleanup
    exit "$1"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

log "플랜트 $PLANT  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  노드 인자: ${NODE_ARGS[*]:-(D-7 60/90)}  상한 ${MAX_MIN}분"

# 1. 엔드포인트
spawn "$OUT/endpoint.pid" "$OUT/endpoint.log" bash "$SIM_ROOT/scripts/run_endpoint.sh"
EP_PORT="${ENDPOINT_PORT:-10000}"     # run_endpoint.sh 와 같은 변수 (Unity 프리팹 포트와 맞아야 한다)
for _ in $(seq 1 50); do ss -ltn | grep -qE "[:.]${EP_PORT}\b" && break; sleep 0.2; done
ss -ltn | grep -qE "[:.]${EP_PORT}\b" || { RESULT="endpoint_fail"; log "!! 엔드포인트가 포트 ${EP_PORT} 를 열지 않았다"; exit 1; }
log "엔드포인트 준비"

# 2. Unity 플랜트
if [ "$PLANT" = editor ]; then
    log "에디터 플랜트: 다른 터미널에서 bash scripts/open_editor.sh command 를 실행하세요 (에디터가 열리고 Command 모드로 Play 될 때까지 최대 10분 대기)"
    OV_TIMEOUT=600
else
OV_TIMEOUT=180
log "Unity 플랜트 시작 (M2PlantRun)"
spawn "$OUT/unity.pid" "$OUT/unity.stdout" env M2_STOP_FILE="$STOP_FILE" M2_LOG_DIR="$OUT" M2_MAX_SECONDS=$((MAX_MIN * 60 + 120)) \
    bash -c "cd '$PROJECT' && exec '$UNITY_BIN' -batchmode -nographics -projectPath '$PROJECT' -runTests -testPlatform PlayMode -testFilter M2PlantRun -testResults '$OUT/test_results.xml' -logFile '$OUT/unity.log'"
fi

# 3. 상태가 overview 로 나오는지 (Unity 기동·컴파일 포함 최대 180 s)
if ! python3 -B "$SIM_ROOT/scripts/check_overview.py" --timeout "$OV_TIMEOUT" > "$OUT/check_overview_1.txt" 2>&1; then
    cat "$OUT/check_overview_1.txt"; RESULT="plant_not_ready"; log "!! 플랜트 상태가 overview 로 오지 않는다"; exit 1
fi
log "플랜트 overview 확인 (1차)"; tail -1 "$OUT/check_overview_1.txt"

# 4. ROS 쪽 주기 측정
spawn "$OUT/ratemon.pid" "$OUT/ratemon.log" python3 -B "$SIM_ROOT/scripts/ros_rate_monitor.py" --out "$OUT/ros_rate.json"

# 5. 딸기 노드
log "딸기 노드 시작"
spawn "$OUT/nodes.pid" "$OUT/nodes.stdout" env M2_LOG_DIR="$OUT" bash "$SIM_ROOT/scripts/run_strawberry_nodes.sh" "${NODE_ARGS[@]}"
for _ in $(seq 1 900); do
    grep -q "전부 정합\|트리거하지 말고" "$OUT/startup_check.txt" 2>/dev/null && break
    kill -0 "$(cat "$OUT/nodes.pid")" 2>/dev/null || break
    sleep 0.5
done
if ! grep -q "전부 정합" "$OUT/startup_check.txt" 2>/dev/null; then
    cat "$OUT/startup_check.txt" 2>/dev/null; RESULT="nodes_startup_fail"; log "!! 딸기 노드 기동 대조 실패"; exit 1
fi
log "딸기 노드 기동 대조 전부 정합"

# 6. overview 재확인(플래너 기동 직후 set_position 이 낸 /joint_command 가 팔을 움직였는지) → 트리거
sleep 3
if ! python3 -B "$SIM_ROOT/scripts/check_overview.py" --timeout 10 > "$OUT/check_overview_2.txt" 2>&1; then
    cat "$OUT/check_overview_2.txt"; RESULT="overview_lost_before_trigger"; log "!! 트리거 직전 overview 아님 — 트리거하지 않는다"; exit 1
fi
log "플랜트 overview 확인 (2차) → 트리거"
timeout 30 ros2 service call /strawberry/scan/start std_srvs/srv/Trigger > "$OUT/trigger.txt" 2>&1
cat "$OUT/trigger.txt" | tail -3 | tee -a "$OUT/orchestrator.log"

# 7. 끝 대기
DEADLINE=$(( $(date +%s) + MAX_MIN * 60 ))
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
    if grep -q "START_REJECTED" "$OUT/scan.log" 2>/dev/null; then RESULT="start_rejected"; break; fi
    if grep -q "\bSCAN_COMPLETE\b" "$OUT/scan.log" 2>/dev/null; then RESULT="scan_complete"; sleep 5; break; fi
    if grep -q "EXEC_TIMEOUT" "$OUT/scan.log" 2>/dev/null; then RESULT="exec_timeout"; sleep 5; break; fi
    if [ "$PLANT" = batch ]; then kill -0 "$(cat "$OUT/unity.pid")" 2>/dev/null || { RESULT="unity_exited"; break; }; fi
    sleep 2
done
[ "$RESULT" = "unknown" ] && RESULT="time_cap_${MAX_MIN}min"
log "종료 사유: $RESULT"
exit 0
