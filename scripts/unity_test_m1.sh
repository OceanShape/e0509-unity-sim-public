#!/usr/bin/env bash
# M1 헤드리스 스모크 테스트 (단계 6): 엔드포인트 + 가짜 퍼블리셔를 띄우고 Unity PlayMode 테스트(M1SmokeTest)를 batchmode 로 돈다.
#
#   bash scripts/unity_test_m1.sh [fake_joint_states.py 인자…]      기본: --hold 1 --ramp 1 --limit-test
#
# 결과: logs/m1/<STAMP>/{endpoint.log,source.log,test_results.xml,smoke_report.json,limit_violations.csv,limit_near.csv,unity.log}
# 종료 코드: Unity 테스트 러너의 코드 (0 통과, 2 실패, 3 러너 오류). 에디터가 프로젝트를 열고 있으면 unity_batch.sh 처럼 멈춘다.
# Ctrl-C(또는 TERM)는 엔드포인트·소스를 정리하고 끝낸다(130/143). 정리 중 Ctrl-C 는 무시한다.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
PROJECT="$SIM_ROOT/UnityProject"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$SIM_ROOT/logs/m1/$STAMP"
mkdir -p "$OUT"
# 소스: 기본 fake. `bag <경로> [ros2 bag play 인자…]` 면 Isaac 런 bag 을 한 번 재생하고 M1_OBSERVE_SECONDS 동안 관찰한다.
SRC=fake
if [ "${1:-}" = "bag" ]; then SRC=bag; shift; BAG_PATH="${1:-$HOME/bags/run1}"; shift || true; BAG_ARGS=("$@"); fi
FAKE_ARGS=(); [ "$SRC" = fake ] && { FAKE_ARGS=("$@"); [ ${#FAKE_ARGS[@]} -eq 0 ] && FAKE_ARGS=(--hold 1 --ramp 1 --limit-test); }

for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        echo "!! Unity 가 이미 $PROJECT 를 열고 있다 (pid $pid). 에디터를 닫은 뒤 다시 실행." >&2; exit 1
    fi
done

RUN_M1_PID=""
cleanup() {
    # 정리는 끝까지 한다: 여기서부터 Ctrl-C·TERM 은 무시한다(run_m2_headless.sh 와 같은 방식).
    trap '' INT TERM
    trap - EXIT
    [ -z "$RUN_M1_PID" ] && return
    # run_m1.sh 는 setsid 로 띄워 자기 프로세스 그룹의 리더다. 그룹 전체(엔드포인트·퍼블리셔·tee)에 TERM 을 보낸다.
    # INT 는 안 된다: 스크립트가 백그라운드로 띄운 자식은 SIGINT 가 무시된 채 시작해 trap 이 걸리지 않는다.
    kill -TERM -- "-$RUN_M1_PID" 2>/dev/null || kill -TERM "$RUN_M1_PID" 2>/dev/null || true
    for _ in $(seq 1 40); do kill -0 "$RUN_M1_PID" 2>/dev/null || break; sleep 0.25; done
    if kill -0 "$RUN_M1_PID" 2>/dev/null; then
        echo "[unity_test_m1] run_m1 그룹이 10초 안에 안 끝나 KILL" >&2
        kill -KILL -- "-$RUN_M1_PID" 2>/dev/null || true
    fi
    wait "$RUN_M1_PID" 2>/dev/null || true
    # 남은 것이 있으면 보고만 한다(다른 세션의 프로세스일 수 있으니 건드리지 않는다)
    pgrep -af "[f]ake_joint_states.py|[d]efault_server_endpoint" >/dev/null && echo "[unity_test_m1] 경고: 다른 퍼블리셔/엔드포인트가 아직 떠 있다 — 결과가 섞일 수 있다" >&2
}
UNITY_RAN=0
collect() {  # Unity 가 logs/ 에 쓴 결과를 이 실행 폴더로
    for f in smoke_report.json mirror_trace.csv; do [ -f "$SIM_ROOT/logs/m1/$f" ] && mv "$SIM_ROOT/logs/m1/$f" "$OUT/"; done
    for f in limit_violations.csv limit_near.csv; do [ -f "$SIM_ROOT/logs/$f" ] && cp "$SIM_ROOT/logs/$f" "$OUT/"; done
}
on_signal() {  # Ctrl-C·TERM: 정리하고 그 자리에서 끝낸다
    echo "[unity_test_m1] 중단 — 엔드포인트·소스 정리 (정리 중 Ctrl-C 는 무시한다)" >&2
    cleanup
    # batchmode Unity 는 Ctrl-C 를 무시하고 시험을 끝까지 돈 뒤 끝난다. 그 뒤에야 이 trap 이 돌므로 결과를 이 폴더로 옮긴다
    # (logs/m1/ 에 남겨 두면 다음 실행이 자기 결과로 옮긴다).
    [ "$UNITY_RAN" = 1 ] && collect
    echo "[unity_test_m1] 중단됨 — 결과 폴더: ${OUT#"$SIM_ROOT"/}" >&2
    exit "$1"
}
trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

# 0) 이전 실행이 남긴 퍼블리셔/엔드포인트가 있으면 결과가 섞인다(두 소스 → 120 Hz, 가짜 속도 위반). 미리 막는다.
if pgrep -af "[f]ake_joint_states.py|[d]efault_server_endpoint" >/dev/null; then
    echo "!! 가짜 퍼블리셔 또는 엔드포인트가 이미 떠 있다. 먼저 끄고 다시 실행:" >&2
    pgrep -af "[f]ake_joint_states.py|[d]efault_server_endpoint" >&2
    exit 1
fi

# 1) 엔드포인트 + 가짜 퍼블리셔 (run_m1.sh 가 둘을 띄운다). setsid 로 새 프로세스 그룹에 두어 한 번에 끌 수 있게 한다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ "$SRC" = bag ]; then
    M1_BAG_ONCE=1 M1_LOG_DIR="$OUT" setsid bash "$SIM_ROOT/scripts/run_m1.sh" bag "$BAG_PATH" "${BAG_ARGS[@]}" > "$OUT/run_m1.log" 2>&1 &
else
    M1_LOG_DIR="$OUT" setsid bash "$SIM_ROOT/scripts/run_m1.sh" fake "${FAKE_ARGS[@]}" > "$OUT/run_m1.log" 2>&1 &
fi
RUN_M1_PID=$!
for _ in $(seq 1 60); do grep -qE "가짜 퍼블리셔 시작|bag 재생" "$OUT/run_m1.log" 2>/dev/null && break; sleep 0.25; done
if ! grep -qE "가짜 퍼블리셔 시작|bag 재생" "$OUT/run_m1.log"; then echo "!! run_m1.sh 가 뜨지 않았다:" >&2; cat "$OUT/run_m1.log" >&2; exit 1; fi
echo "[unity_test_m1] 엔드포인트·소스($SRC) 준비 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, 인자: ${FAKE_ARGS[*]:-}${BAG_PATH:-} ${BAG_ARGS[*]:-})"

# 2) Unity PlayMode 테스트
echo "[unity_test_m1] Unity 테스트 시작 → $OUT/unity.log"
START=$(date +%s)
UNITY_RAN=1
( cd "$PROJECT" && "$UNITY_BIN" -batchmode -nographics -projectPath "$PROJECT" \
    -runTests -testPlatform PlayMode -testFilter M1SmokeTest -testResults "$OUT/test_results.xml" -logFile "$OUT/unity.log" )
RC=$?
echo "[unity_test_m1] Unity 종료 rc=$RC ($(( $(date +%s) - START ))s)"

# 3) 결과 모으기
collect
if [ -f "$OUT/test_results.xml" ]; then
    grep -oE 'test-run [^>]*' "$OUT/test_results.xml" | head -1 | grep -oE '(result|total|passed|failed)="[^"]*"' | tr '\n' ' '; echo
    grep -oE '<failure>.*</failure>|<message><!\[CDATA\[[^]]*' "$OUT/test_results.xml" | head -5
fi
[ -f "$OUT/smoke_report.json" ] && { echo "--- smoke_report.json"; cat "$OUT/smoke_report.json"; }
grep -nE '^\[(M1SmokeTest|JointStateMirror|JointLimitMonitor|TcpTrail)\]|error CS|Scripts have compiler errors' "$OUT/unity.log" | head -40
echo "[unity_test_m1] 결과 폴더: ${OUT#"$SIM_ROOT"/}"
exit $RC
