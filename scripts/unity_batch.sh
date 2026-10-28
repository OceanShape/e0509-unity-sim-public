#!/usr/bin/env bash
# Unity 에디터를 batchmode 로 한 번 돌린다 (단계 4 이후 공용).
#
#   bash scripts/unity_batch.sh <번호> <이름> [-executeMethod Class.Method] [그 밖의 Unity 인자…]
#   예) bash scripts/unity_batch.sh 03 configure -executeMethod SimSetup.ConfigureProject
#       bash scripts/unity_batch.sh 04 build_scene -executeMethod SimSetup.BuildScene
#       bash scripts/unity_batch.sh 02 resolve_packages            (인자 없이 열고 닫기 = 패키지 해석·에셋 임포트)
#
# - 전체 로그: logs/unity/<번호>_<이름>.log (크고 반복적이라 git 에서 제외)
# - 발췌 로그: logs/step4/<번호>_<이름>.extract.log (오류·경고·우리 스크립트 출력·종료 줄만, 커밋 대상)
# - 에디터가 이 프로젝트를 열고 있으면 batchmode 가 못 열므로 먼저 확인하고 멈춘다.
# - 종료 코드: Unity 의 종료 코드. 로그에 "Scripts have compiler errors" 나 -executeMethod 예외가 있으면 1.
set -u
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
[ -f "$SIM_ROOT/scripts/env.sh" ] && source "$SIM_ROOT/scripts/env.sh"
UNITY_BIN="${UNITY_BIN:-}"
PROJECT="$SIM_ROOT/UnityProject"
EXTRACT_DIR="${EXTRACT_DIR:-$SIM_ROOT/logs/step4}"

if [ $# -lt 2 ]; then
    echo "사용법: $0 <번호> <이름> [Unity 인자…]" >&2
    exit 2
fi
NUM="$1"; NAME="$2"; shift 2

if [ -z "$UNITY_BIN" ] || [ ! -x "$UNITY_BIN" ]; then
    echo "!! UNITY_BIN 이 없다. scripts/check_env.sh 를 먼저 실행" >&2
    exit 1
fi
# 같은 프로젝트를 연 에디터(또는 다른 batchmode)가 있으면 멈춘다.
# 명령줄 전체를 grep 하면 이 스크립트를 부른 셸(인자에 프로젝트 경로가 있음)까지 잡히므로, Unity 실행 파일 프로세스만 본다.
BUSY=""
for pid in $(pgrep -f "^${UNITY_BIN}( |$)" 2>/dev/null); do
    if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qi -- "-projectpath $PROJECT"; then
        BUSY="$BUSY $pid"
    fi
done
if [ -n "$BUSY" ]; then
    echo "!! Unity 가 이미 $PROJECT 를 열고 있다 (pid$BUSY). 에디터를 닫은 뒤 다시 실행." >&2
    exit 1
fi
if [ -f "$PROJECT/Temp/UnityLockfile" ]; then
    # 위에서 이 프로젝트를 연 Unity 프로세스가 없음을 확인했다. 컴파일 오류 등으로 비정상 종료한 batchmode 가 남긴 것이므로 지운다.
    if pgrep -f "^${UNITY_BIN}( |$)" >/dev/null; then
        echo "!! $PROJECT/Temp/UnityLockfile 이 있고 다른 Unity 프로세스가 떠 있다. 확인 후 다시 실행" >&2
        exit 1
    fi
    echo "[unity_batch] 남은 Temp/UnityLockfile 삭제 (Unity 프로세스 없음)"
    rm -f "$PROJECT/Temp/UnityLockfile"
fi

mkdir -p "$SIM_ROOT/logs/unity" "$EXTRACT_DIR"
LOG="$SIM_ROOT/logs/unity/${NUM}_${NAME}.log"
EXTRACT="$EXTRACT_DIR/${NUM}_${NAME}.extract.log"

echo "[unity_batch] $NAME 시작 → $LOG"
echo "[unity_batch] 인자: -batchmode ${UNITY_GRAPHICS:+(그래픽) }-projectPath $PROJECT $* -quit"
START=$(date +%s)
# URDF Importer 의 STL 리더가 "Assets/…" 상대 경로를 직접 연다(StlImporter.cs:38). cwd 를 프로젝트 루트로 맞춘다.
# UNITY_GRAPHICS=1 이면 -nographics 를 빼서 GPU 로 렌더한다(SimEnvironment.RenderPreview 용)
NOGFX=(-nographics); [ "${UNITY_GRAPHICS:-0}" = "1" ] && NOGFX=()
( cd "$PROJECT" && "$UNITY_BIN" -batchmode "${NOGFX[@]}" -projectPath "$PROJECT" "$@" -quit -logFile "$LOG" )
RC=$?
END=$(date +%s)

{
    echo "# $(date '+%F %T')  unity_batch $NUM $NAME  rc=$RC  $((END-START))s"
    echo "# 인자: $*"
    echo "# 전체 로그: ${LOG#"$SIM_ROOT"/}"
    echo
    # 우리 스크립트의 출력([SimSetup]·[M1SmokeTest]·런타임 스크립트), 컴파일 오류, 예외 첫 줄, 임포터 경고, 종료 줄만 남긴다.
    # 스택 프레임(Unity.Robotics.…, UnityEngine.…, "(at …)", "(Filename: …)")과 매 실행 반복되는 라이선스 노이즈는 뺀다.
    grep -nE '^\[(Sim|M1Smoke|JointState|JointLimit|TcpTrail)|error CS|[A-Za-z]+Exception: |Scripts have compiler errors|Aborting batchmode|Exiting batchmode|Asset Not Found|Interrupted|URDF file and resources|Couldn.t create a Convex|Unable to create mesh|Missing Asset|Scene.*(saved|Saved)' "$LOG" \
      | grep -vE 'Performance\]|Start importing|ErrorLog|LogAssemblyErrors|^[0-9]+:(Unity\.|UnityEngine\.|UnityEditor\.|SimSetup:|System\.|NUnit\.|\(at |\(Filename| at |#[0-9])' \
      | head -400
} > "$EXTRACT"

echo "[unity_batch] 종료 rc=$RC ($((END-START))s). 발췌 → ${EXTRACT#"$SIM_ROOT"/}"
if grep -qE 'Scripts have compiler errors|Aborting batchmode' "$LOG"; then
    echo "!! 컴파일 오류 또는 batchmode 중단:" >&2
    grep -nE 'error CS|Aborting batchmode' "$LOG" | head -20 >&2
    exit 1
fi
if grep -qE '^\[SimSetup\] FAIL' "$LOG"; then
    echo "!! SimSetup 이 실패를 보고했다:" >&2
    grep -nE '^\[SimSetup\] FAIL' "$LOG" >&2
    exit 1
fi
exit $RC
