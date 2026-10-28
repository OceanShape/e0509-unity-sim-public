#!/usr/bin/env bash
# 단계 0 — 환경 점검.
#   - 결과를 stdout 으로 낸다(docs/PROGRESS.md 맨 위에 붙임).
#   - Unity 에디터 경로를 scripts/env.sh 의 UNITY_BIN 으로 저장한다.
#   - 종료 코드 = FAIL 항목 수.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAIL=0
ok()   { printf '  [OK]   %s\n' "$*"; }
ng()   { printf '  [FAIL] %s\n' "$*"; FAIL=$((FAIL+1)); }
info() { printf '  [..]   %s\n' "$*"; }

echo "## 환경 점검 — $(date '+%Y-%m-%d %H:%M') @ $(hostname)"
echo '```'

# 1. 디스크 여유 (기준 15GB)
avail_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
if [ "${avail_gb:-0}" -ge 15 ]; then ok "디스크 여유 ${avail_gb}GB (기준 15GB)"; else ng "디스크 여유 ${avail_gb}GB < 15GB"; fi

# 2. git, unityhub
for b in git unityhub; do
  if p=$(command -v "$b" 2>/dev/null); then ok "$b: $p"; else ng "$b 없음 (H1)"; fi
done

# 3. Unity 에디터 2022.3.x (최신 패치 하나 선택) → scripts/env.sh. 다른 곳에 있으면 UNITY_BIN=/경로/Editor/Unity 로 준다.
UNITY_BIN="${UNITY_BIN:-$(ls -d "$HOME"/Unity/Hub/Editor/2022.3.*/Editor/Unity 2>/dev/null | sort -V | tail -1)}"
if [ -n "$UNITY_BIN" ] && [ -x "$UNITY_BIN" ]; then
  ok "Unity 에디터: $UNITY_BIN"
  STRAWBERRY_WS="${STRAWBERRY_WS:-$HOME/strawberry_grasp_environment}"
  cat > "$ROOT/scripts/env.sh" <<ENV
# check_env.sh 가 자동 생성. 손으로 고치지 말고 check_env.sh 를 다시 실행할 것.
export SIM_ROOT="$ROOT"
export UNITY_BIN="$UNITY_BIN"
export STRAWBERRY_WS="$STRAWBERRY_WS"   # 딸기 리포(읽기 전용)
ENV
  info "scripts/env.sh 저장 (UNITY_BIN, SIM_ROOT, STRAWBERRY_WS)"
else
  ng "Unity 에디터 없음: ${UNITY_BIN:-~/Unity/Hub/Editor/2022.3.*/Editor/Unity} (H2). 다른 곳에 있으면 UNITY_BIN=/경로/Editor/Unity bash scripts/check_env.sh"
fi
# 라이선스 파일 존재 여부(최종 판단은 단계 4 createProject)
if [ -f "$HOME/.local/share/unity3d/Unity/Unity_lic.ulf" ] || ls "$HOME"/.config/unity3d/Unity/licenses/*.xml >/dev/null 2>&1; then
  info "Unity 라이선스 파일 있음 (최종 확인은 단계 4 createProject)"
else
  info "Unity 라이선스 파일 없음 → Hub 에서 로그인·Personal 활성화 필요할 수 있음 (H2)"
fi
# 에디터 실행 중인지 (batchmode 전 확인용)
if pgrep -x Unity >/dev/null 2>&1; then info "Unity 에디터 실행 중 — batchmode 작업 전 닫아야 함"; else info "Unity 에디터 실행 중 아님"; fi

# 4. ROS 2 humble
if [ -f /opt/ros/humble/setup.bash ]; then
  if ( set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; [ "${ROS_DISTRO:-}" = humble ] ); then
    ok "ROS 2: /opt/ros/humble/setup.bash → ROS_DISTRO=humble"
  else
    ng "/opt/ros/humble/setup.bash 소싱 후 ROS_DISTRO != humble"
  fi
else
  ng "/opt/ros/humble/setup.bash 없음"
fi
info "현재 셸 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<미설정 → 0>}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<미설정 → rmw_fastrtps_cpp>}"

# 5. 딸기 워크스페이스(읽기 전용)
SW="${STRAWBERRY_WS:-$HOME/strawberry_grasp_environment}"
if [ -f "$SW/install/setup.bash" ]; then ok "딸기 ws: $SW/install/setup.bash"; else ng "딸기 ws setup.bash 없음: $SW"; fi

# 6. nvidia-smi
if out=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>&1); then
  ok "nvidia-smi: $out"
else
  ng "nvidia-smi 실패: $out"
fi

echo '```'
if [ "$FAIL" -eq 0 ]; then echo "결과: 전부 통과"; else echo "결과: FAIL ${FAIL}건"; fi
exit "$FAIL"
