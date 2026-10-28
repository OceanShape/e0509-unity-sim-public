#!/usr/bin/env bash
# 딸기 수확 노드 4개를 딸기 리포 밖에 로그를 남기며 띄운다 (단계 6-4·7, CONTRACT §5-1, §9-4 D-5·D-7).
#
#   bash scripts/run_strawberry_nodes.sh              띄운다 (플랜트 = Unity Play 또는 Isaac 이 먼저 떠 있어야 한다, CONTRACT §5-4)
#   bash scripts/run_strawberry_nodes.sh --kill       남은 노드만 정리
#   bash scripts/run_strawberry_nodes.sh --no-check   check_params.py 게이트 생략
#   bash scripts/run_strawberry_nodes.sh --isaac-speeds   스캔 MoveJoint 를 딸기 run_nodes.sh 와 같은 120/180 으로 (기본은 D-7 의 60/90)
#
# 딸기 리포 scripts/run_nodes.sh (커밋 11014db, 2026-10-12) 와 같은 노드·순서·인자로 띄운다. 다른 점은 셋뿐이다.
#   1. 로그는 이 리포 logs/m2/<STAMP>/{vision,bridge,planner,scan}.log (M2_LOG_DIR 로 바꿀 수 있다). run_logs/ 를 만들지 않는다.
#   2. STRAWBERRY_RUNTIME_LOG_ROOT=logs/runtime (플래너 JSONL), PYTHONDONTWRITEBYTECODE=1 (__pycache__) — D-5
#   3. 스캔·overview 복귀 MoveJoint 60 °/s·90 °/s² (실기 원본, D-7). 기동 대조의 기대 문자열도 그에 맞춘다.
# 노드 코드는 딸기 리포의 것을 패키지/파일 이름으로 참조만 한다. 파라미터는 --ros-args 로만 바꾼다.
# 끝날 때 딸기 리포의 git 상태(무시 파일 포함)를 시작 때와 비교해 새로 생긴 파일이 있으면 경고한다.
# Ctrl-C(또는 TERM)는 노드를 정리하고 끝낸다(종료 코드 130/143). 정리 중 Ctrl-C 는 무시한다.
# 노드와 함께 scripts/hud_relay.py 를 띄운다: 노드 계측이 쓰는 /tmp/harvest_hud_*.json 을 딸기 HUD 모듈로 합쳐
# Unity 화면(HUD·보드 하이라이트·손목 카메라 미니맵)이 읽을 /tmp/e0509_hud_view.json 을 10 Hz 로 쓴다(단계 8 화면).
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
REPO="${STRAWBERRY_WS:-$HOME/strawberry_grasp_environment}"

# 로그 파일 이름에 노드 이름을 쓰지 않는다 — pkill -f 패턴에 tee 가 걸린다(딸기 run_nodes.sh 와 같은 이유).
NODE_NAMES=(fake_vision_node sim_executor_bridge_node curobo_planner_node scan_executor_node status_monitor_node)
NODE_PAT="$(IFS='|'; echo "${NODE_NAMES[*]}")"

kill_nodes() {
    for n in "${NODE_NAMES[@]}"; do pkill -f "$n" >/dev/null 2>&1; done
    sleep 2
    # planner 는 MotionGen warmup 중 SIGTERM 을 늦게 받는다 → -9
    for n in "${NODE_NAMES[@]}"; do pkill -9 -f "$n" >/dev/null 2>&1; done
    sleep 1
}

KILL_ONLY=0; NO_CHECK=0; SCAN_VEL=60.0; SCAN_ACC=90.0
for arg in "$@"; do
    case "$arg" in
        --kill)         KILL_ONLY=1 ;;
        --no-check)     NO_CHECK=1 ;;
        --isaac-speeds) SCAN_VEL=120.0; SCAN_ACC=180.0 ;;
        *) echo "!! 모르는 인자: $arg  (가능: --kill, --no-check, --isaac-speeds)" >&2; exit 1 ;;
    esac
done

if [ "$KILL_ONLY" = "1" ]; then
    echo "[run_strawberry_nodes] 노드 정리 중..."
    kill_nodes
    left="$(pgrep -af "$NODE_PAT")"
    if [ -n "$left" ]; then echo "!! 아직 남았다:"; echo "$left"; exit 1; fi
    echo "[run_strawberry_nodes] 정리 완료."
    exit 0
fi

[ -f "$REPO/install/setup.bash" ] || { echo "!! 딸기 ws 없음: $REPO/install/setup.bash" >&2; exit 1; }
LEFT="$(pgrep -af "$NODE_PAT")"
if [ -n "$LEFT" ]; then
    echo "!! 이전 실행의 노드가 살아 있다. 새 노드와 토픽·서비스가 겹친다." >&2
    echo "$LEFT" >&2
    echo "   정리:  bash scripts/run_strawberry_nodes.sh --kill" >&2
    exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="${M2_LOG_DIR:-$SIM_ROOT/logs/m2/$STAMP}"
mkdir -p "$LOGDIR" "$SIM_ROOT/logs/runtime"

export PYTHONDONTWRITEBYTECODE=1                                  # D-5: __pycache__ 를 딸기 리포에 만들지 않는다
export STRAWBERRY_RUNTIME_LOG_ROOT="$SIM_ROOT/logs/runtime"       # D-5: 플래너 JSONL
export PYTHONUNBUFFERED=1
export HARVEST_RECORD_CAMERA=0                                    # 녹화 카메라 신호 끔(Isaac 표시 스크립트용, Unity 와 무관)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"                        # CONTRACT §7
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# 딸기 리포 상태 스냅샷(무시 파일 포함). 끝에서 비교한다.
snapshot() { git --no-optional-locks -C "$REPO" status --porcelain --ignored 2>/dev/null | sort; }
snapshot > "$LOGDIR/strawberry_status_before.txt"

# ── 설정 정합 게이트 (USD·yaml 을 텍스트로 읽기만 한다) ──
if [ "$NO_CHECK" = "0" ]; then
    if ! ( cd "$REPO" && python3 -B check_params.py ) > "$LOGDIR/check_params.log" 2>&1; then
        cat "$LOGDIR/check_params.log"
        echo "!! check_params.py 실패. 강행하려면 --no-check" >&2
        exit 1
    fi
    echo "[run_strawberry_nodes] check_params.py 통과"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || { echo "!! /opt/ros/humble 소싱 실패" >&2; exit 1; }
# shellcheck disable=SC1091
source "$REPO/install/setup.bash" || { echo "!! 딸기 install 소싱 실패" >&2; exit 1; }
set -u

# HUD 스냅샷 잔재 제거 (/tmp, 딸기 리포 밖. run_nodes.sh 와 같다)
rm -f /tmp/harvest_hud_*.json
HUD_VIEW="${E0509_HUD_VIEW:-/tmp/e0509_hud_view.json}"
rm -f "$HUD_VIEW"
# HUD 중계 (딸기 HUD 모듈을 import 만 한다. -B 로 __pycache__ 없음)
E0509_HUD_VIEW="$HUD_VIEW" python3 -B "$SIM_ROOT/scripts/hud_relay.py" > "$LOGDIR/hud_relay.log" 2>&1 &
RELAY_PID=$!

cleanup() {
    # 정리는 끝까지 한다: 여기서부터 Ctrl-C·TERM 은 무시한다(run_m2_headless.sh 와 같은 방식).
    # 중간에 끊기면 TERM 을 늦게 받는 플래너(-9 단계)와 딸기 리포 상태 비교(D-5)가 빠진다.
    trap '' INT TERM
    trap - EXIT
    echo
    echo "[run_strawberry_nodes] 종료 중 — 노드 정리 (정리 중 Ctrl-C 는 무시한다)"
    kill_nodes
    # 중계는 노드 뒤에 끈다. 뷰 파일은 남긴다 — Isaac HUD 처럼 마지막 결과가 화면에 남고, Unity 는 파일 시각으로 램프를 끈다
    kill "$RELAY_PID" 2>/dev/null; wait "$RELAY_PID" 2>/dev/null
    left="$(pgrep -af "$NODE_PAT")"
    [ -n "$left" ] && { echo "!! 아직 남은 노드:"; echo "$left"; }
    snapshot > "$LOGDIR/strawberry_status_after.txt"
    NEW="$(comm -13 "$LOGDIR/strawberry_status_before.txt" "$LOGDIR/strawberry_status_after.txt")"
    if [ -n "$NEW" ]; then
        echo "!! 딸기 리포에 새로 생긴/바뀐 항목이 있다 (D-5 위반 후보):"
        echo "$NEW" | tee "$LOGDIR/strawberry_status_new.txt"
    else
        echo "[run_strawberry_nodes] 딸기 리포 상태 변화 없음 (무시 파일 포함)"
    fi
    echo "[run_strawberry_nodes] 로그: ${LOGDIR#"$SIM_ROOT"/}"
}
on_signal() {  # Ctrl-C·TERM: 정리하고 그 자리에서 끝낸다
    cleanup
    exit "$1"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

echo "[run_strawberry_nodes] 로그 ${LOGDIR#"$SIM_ROOT"/}  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  스캔 MoveJoint ${SCAN_VEL}/${SCAN_ACC}"

# ── 기동 (run_nodes.sh 와 같은 순서·인자) ──
( cd "$REPO" && exec stdbuf -oL -eL ros2 run strawberry_sim_core fake_vision_node ) \
    > >(tee "$LOGDIR/vision.log" | stdbuf -oL sed 's/^/[vision]  /') 2>&1 &

( cd "$REPO" && exec stdbuf -oL -eL ros2 run strawberry_sim_core sim_executor_bridge_node ) \
    > >(tee "$LOGDIR/bridge.log" | stdbuf -oL sed 's/^/[bridge]  /') 2>&1 &

# bridge 의 cuRobo IK 초기화를 기다린다 (최대 120초, CUDA 경합 회피)
for _ in $(seq 1 240); do
    grep -q "cuRobo IK Solver successfully initialized" "$LOGDIR/bridge.log" 2>/dev/null && break
    sleep 0.5
done

# planner 는 flat import 구조라 scripts/ 에서 실행한다(src 원본 직접 실행, CONTRACT §5-1 4번)
( cd "$REPO/src/strawberry_motion/scripts" && exec stdbuf -oL -eL python3 curobo_planner_node.py --ros-args \
    -p tool_model_profile:=legacy_160mm \
    -p ee_to_tcp_offset_m:=0.236 \
    -p enable_open_stem_descent:=true \
    -p enable_straight_reverse_retreat:=true \
    -p pick_target_z_bias_m:=0.035 \
    -p allow_generated_tray_slot_release:=true \
    -p enable_marker_place_sequence:=true \
    -p use_taught_slot0_place_reference:=true \
    -p taught_place_via_tray_view:=false \
    -p execute_marker_place_release:=true \
    -p hold_after_taught_slot0_place:=false \
    -p hold_on_place_failure:=false \
    -p taught_slot_sequence:=0,1,3,4,6,7,9,10 \
) > >(tee "$LOGDIR/planner.log" | stdbuf -oL sed 's/^/[planner] /') 2>&1 &

( cd "$REPO" && exec stdbuf -oL -eL python3 -m strawberry_motion.execution.scan_executor_node --ros-args \
    -p execute_motion:=true \
    -p target_cell:=all \
    -p overview_prescan:=true \
    -p scan_dwell_sec:=3.0 \
    -p subdivide_min_candidates:=3 \
    -p scan_movej_vel_deg_s:=$SCAN_VEL -p scan_movej_acc_deg_s2:=$SCAN_ACC \
    -p overview_return_vel_deg_s:=$SCAN_VEL -p overview_return_acc_deg_s2:=$SCAN_ACC \
) > >(tee "$LOGDIR/scan.log" | stdbuf -oL sed 's/^/[scan]    /') 2>&1 &

echo
echo "[run_strawberry_nodes] cuRobo Planner warmup 대기 중 (수십 초)..."
READY=0
for _ in $(seq 1 600); do
    grep -q "cuRobo Planner Ready!" "$LOGDIR/planner.log" 2>/dev/null && { READY=1; break; }
    sleep 0.5
done

FAILS=0
need() {  # need <로그파일> <패턴> <설명>
    if grep -q "$2" "$LOGDIR/$1" 2>/dev/null; then printf '  OK   %s\n' "$3"
    else printf '  !!   %s   (없음: %s → %s)\n' "$3" "$2" "$1"; FAILS=$((FAILS + 1)); fi
}
SPEED_LINE="MoveJoint speeds: scan vel=${SCAN_VEL} acc=${SCAN_ACC}, overview return vel=${SCAN_VEL} acc=${SCAN_ACC}"
{
echo "──────────────────────────────────────────────────────────"
echo " 기동 로그 대조 (딸기 run_nodes.sh 와 같은 항목, 속도만 D-7)"
echo "──────────────────────────────────────────────────────────"
need bridge.log  "MOVELINE_COLLISION_WORLD:"            "MoveLine 충돌월드 로드"
need bridge.log  "tool_tcp_offset=236mm"                "브릿지 TCP 오프셋 236mm"
need bridge.log  "arm_arrival_tol=0.30deg"              "도착 판정 허용오차 0.30deg"
need planner.log "EE_TO_TCP_OFFSET_OVERRIDE"            "플래너 TCP 오프셋 160→236mm"
need planner.log "open_stem_descent=True"               "열린 조우 하강 단계"
need planner.log "straight_reverse_retreat=True"        "진입 역순 후퇴 단계"
need planner.log "taught_place_via_tray_view=False"     "배치 이송 = 실기 시연과 같은 직행"
need planner.log "slot_sequence=\[0, 1, 3, 4, 6, 7, 9, 10\]" "배치 슬롯 0,1,3,4,6,7,9,10"
need planner.log "orthogonalize_taught_grid=False"      "배치 격자 = 실기 티칭 그대로"
need planner.log "taught_grid_pitch_override_m=0.0000"  "배치 격자 간격 = 실기 티칭"
need planner.log "taught_grid_shift_y_m=0.0000"         "배치 격자 평행이동 없음"
need scan.log    "scan_executor_node ready"             "scan_executor 기동"
need scan.log    "SUBDIVIDE_IK_READY min_candidates=3"  "적응 분할 IK 솔버"
need scan.log    "$SPEED_LINE"                          "스캔·overview 복귀 MoveJoint ${SCAN_VEL}/${SCAN_ACC}"
[ "$READY" = "1" ] && printf '  OK   %s\n' "cuRobo Planner Ready!" \
                   || { printf '  !!   %s\n' "cuRobo Planner Ready! 가 5분 안에 안 떴다"; FAILS=$((FAILS + 1)); }
echo "──────────────────────────────────────────────────────────"
if [ "$FAILS" -gt 0 ]; then
    echo " 대조 실패 $FAILS 건 — 트리거하지 말고 원인을 잡는다."
else
    echo " 전부 정합. 플랜트(Unity Play 또는 Isaac)가 overview 자세로 /dsr01/joint_states 를 내는지 확인한 뒤:"
    echo "   ros2 service call /strawberry/scan/start std_srvs/srv/Trigger"
fi
echo "──────────────────────────────────────────────────────────"
} | tee "$LOGDIR/startup_check.txt"

wait
