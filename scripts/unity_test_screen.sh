#!/usr/bin/env bash
# 단계 8 화면 시험 (ROS 없이, batchmode): GripperGeometryCheck · GraspFollowTest · HudViewTest · CommandModeTest(회귀)
#
#   bash scripts/unity_test_screen.sh
#
# GripperGeometryCheck  같은 관절각에서 Unity 좌우 손가락 최소 거리 = URDF 순기구학 값(±2 mm)
# GraspFollowTest       Isaac bag 의 첫 픽 관절값·이벤트로 부착·운반·해제·낙하·발행 제외·무시 규칙
# HudViewTest           hud_relay.py 출력(scripts/fixtures/hud_view_sample.json, 10-21 M2 런 끝 상태) 파싱 → 보드 테두리·방위 글자
# 결과: logs/m2/<STAMP>_screen_tests/{test_results.xml,gripper_geometry.json,grasp_follow_test.json,command_mode_report.json,…}
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
PROJECT="$SIM_ROOT/UnityProject"
OUT="$SIM_ROOT/logs/m2/$(date +%Y%m%d_%H%M%S)_screen_tests"
for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        echo "!! Unity 가 이미 $PROJECT 를 열고 있다 (pid $pid). 에디터를 닫은 뒤 다시 실행." >&2; exit 1
    fi
done
mkdir -p "$OUT"
export HUD_VIEW_SAMPLE="${HUD_VIEW_SAMPLE:-$SIM_ROOT/scripts/fixtures/hud_view_sample.json}"
( cd "$PROJECT" && "$UNITY_BIN" -batchmode -nographics -projectPath "$PROJECT" -runTests -testPlatform PlayMode \
    -testFilter "GripperGeometryCheck|GraspFollowTest|HudViewTest|CommandModeTest" \
    -testResults "$OUT/test_results.xml" -logFile "$OUT/unity.log" )
RC=$?
for f in gripper_geometry.json grasp_follow_test.json command_mode_report.json command_mode_trace.csv command_mode_limit_violations.csv command_mode_limit_near.csv; do
    [ -f "$SIM_ROOT/logs/m2/$f" ] && mv "$SIM_ROOT/logs/m2/$f" "$OUT/"
done
grep -oE 'test-run [^>]*' "$OUT/test_results.xml" 2>/dev/null | grep -oE '(result|total|passed|failed|skipped)="[^"]*"' | tr '\n' ' '; echo
grep -oE '<message><!\[CDATA\[[^]]{0,400}' "$OUT/test_results.xml" 2>/dev/null | grep -v "child tests" | head -5
grep -hE '^\[(GripperGeometryCheck|GraspFollowTest|HudViewTest)\]|error CS' "$OUT/unity.log" | head -20
echo "[unity_test_screen] rc=$RC  결과: ${OUT#"$SIM_ROOT"/}"
exit $RC
