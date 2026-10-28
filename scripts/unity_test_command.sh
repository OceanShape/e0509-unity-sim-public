#!/usr/bin/env bash
# M2 Command 모드 수렴 시험 (단계 7): ROS 없이 Unity PlayMode 테스트 CommandModeTest 를 batchmode 로 돈다.
#   bash scripts/unity_test_command.sh
# 결과: logs/m2/<STAMP>_command_mode/{test_results.xml,command_mode_report.json,command_mode_trace.csv,command_mode_limit_*.csv}
# 종료 코드: Unity 테스트 러너의 코드 (0 통과, 2 실패). Unity 전체 로그 unity.log 는 git 제외.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
PROJECT="$SIM_ROOT/UnityProject"
OUT="$SIM_ROOT/logs/m2/$(date +%Y%m%d_%H%M%S)_command_mode"
mkdir -p "$OUT"
for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        echo "!! Unity 가 이미 $PROJECT 를 열고 있다 (pid $pid). 에디터를 닫은 뒤 다시 실행." >&2; exit 1
    fi
done
( cd "$PROJECT" && "$UNITY_BIN" -batchmode -nographics -projectPath "$PROJECT" \
    -runTests -testPlatform PlayMode -testFilter CommandModeTest -testResults "$OUT/test_results.xml" -logFile "$OUT/unity.log" )
RC=$?
for f in command_mode_report.json command_mode_trace.csv command_mode_limit_violations.csv command_mode_limit_near.csv; do
    [ -f "$SIM_ROOT/logs/m2/$f" ] && mv "$SIM_ROOT/logs/m2/$f" "$OUT/"
done
grep -oE 'test-run [^>]*' "$OUT/test_results.xml" 2>/dev/null | grep -oE '(result|passed|failed)="[^"]*"' | tr '\n' ' '; echo
grep -oE '<message><!\[CDATA\[[^]]{0,400}' "$OUT/test_results.xml" 2>/dev/null | grep -v "child tests" | head -3
grep -nE 'error CS|Scripts have compiler errors' "$OUT/unity.log" | head -10
echo "[unity_test_command] rc=$RC  결과: ${OUT#"$SIM_ROOT"/}"
exit $RC
