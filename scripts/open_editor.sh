#!/usr/bin/env bash
# Unity 에디터(화면)를 터미널에서 띄운다 — Hub 를 거치지 않는다. 씬 E0509Sim 을 열고 Play 까지 자동으로 누른다.
#
#   bash scripts/open_editor.sh mirror      M1(Mirror 모드): 받은 /dsr01/joint_states 를 그대로 재생
#   bash scripts/open_editor.sh command     M2(Command 모드): /joint_command 를 따라가고 상태·과실 좌표를 낸다
#   bash scripts/open_editor.sh command --no-play   씬만 열고 Play 는 직접 누른다
#
# 모드는 명령줄 인자(-simMode)로 정하므로 씬을 고치지 않는다(저장할 일 없음).
# 에디터를 닫으면 끝난다. 에디터가 열려 있는 동안 batchmode 스크립트(unity_batch.sh, unity_test_*.sh,
# run_m2_headless.sh 기본 모드)는 같은 프로젝트를 못 연다 — 에디터를 먼저 닫는다.
# 에디터 로그: logs/unity/editor_<모드>.log (git 제외)
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
PROJECT="$SIM_ROOT/UnityProject"
MODE="${1:-}"; shift || true
case "$MODE" in mirror|command) ;; *) echo "사용법: $0 mirror|command [--no-play]" >&2; exit 2 ;; esac
EXTRA=()
[ "${1:-}" = "--no-play" ] && EXTRA+=(-simNoPlay)

for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        echo "!! Unity 가 이미 이 프로젝트를 열고 있다 (pid $pid). 그 창을 쓰거나 닫은 뒤 다시 실행." >&2; exit 1
    fi
done
[ -f "$PROJECT/Temp/UnityLockfile" ] && ! pgrep -f "^${UNITY_BIN}( |$)" >/dev/null && rm -f "$PROJECT/Temp/UnityLockfile"

mkdir -p "$SIM_ROOT/logs/unity"
LOG="$SIM_ROOT/logs/unity/editor_${MODE}.log"
echo "[open_editor] 에디터 시작 (모드 $MODE). 창이 뜨고 씬이 열린 뒤 Play 가 눌린다 — 처음엔 1–2분 걸릴 수 있다."
echo "[open_editor] 로그: ${LOG#"$SIM_ROOT"/}"
cd "$PROJECT" && nohup "$UNITY_BIN" -projectPath "$PROJECT" -simMode "$MODE" "${EXTRA[@]}" \
    -executeMethod SimSetup.OpenSceneAndPlay -logFile "$LOG" > /dev/null 2>&1 &
echo "[open_editor] pid $!  (에디터 창을 닫으면 끝난다)"
