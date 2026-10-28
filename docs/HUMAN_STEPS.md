# 수동 단계 — H3 (에디터 눈 확인) · H4 (Isaac 런 bag 녹화) · H5 (30초 영상)

에디터 확인·Isaac 녹화·영상처럼 손으로 하는 단계(H3–H5)의 실행 안내다. 명령은 모두 이 리포 루트에서 실행한다.
헤드리스 시험(스크립트)으로 이미 확인한 것과, 사람 눈으로만 확인할 수 있는 것을 나눠 적었다.
H3-1 의 bag 재생, H3-2, H4, H5 는 리포에 없는 자원(Isaac 런 bag, 비공개 딸기 리포 워크스페이스)이 필요하다. 이 리포만으로 에디터 화면을 보려면 H3-1 끝의 가짜 퍼블리셔 안내를 따른다.

## H3 — Unity 에디터에서 눈으로 확인

**H3 가 무엇인가.** 지금까지의 Unity 시험은 전부 화면 없이(batchmode) 돌렸다. 숫자(연결·60 Hz·관절값·한계 위반·M2 완주)는 이미 확인했다. 화면 없이는 확인할 수 없는 것이 하나 남는데, 로봇이 **눈으로 보기에** 맞게 서 있느냐다. 링크 메쉬가 제자리에 붙어 있는지, 팔이 곧게 서 있는지, 잡은 딸기가 그리퍼를 따라가는지, HUD·보드 하이라이트·그리퍼 카메라 창이 Isaac 처럼 보이는지를 에디터 화면에서 눈으로 본다.

에디터는 터미널에서 `scripts/open_editor.sh` 로 띄운다. Hub 를 쓰지 않아도 되고, 씬 열기와 ▶ Play 를 스크립트가 대신 누른다. 모드(Mirror/Command)는 명령줄 인자로 정하므로 씬을 고치거나 저장할 일이 없다. 확인이 끝나면 에디터 창을 X 로 닫는다(터미널에서 `kill` 로 끄면 Unity 가 크래시로 처리해 Unity Bug Reporter 창을 띄운다 — 보내지 말고 Cancel). **에디터가 열려 있는 동안에는 다른 Unity 스크립트(unity_batch.sh, unity_test_*.sh, run_m2_headless.sh 기본 모드)가 같은 프로젝트를 못 연다.**

### 화면 조작 (Game 뷰, Play 중)

| 입력 | 동작 |
|---|---|
| 오른쪽 버튼 드래그 | 회전 (화면 중심점 둘레) |
| 가운데 버튼 드래그, 또는 Shift + 오른쪽 드래그 | 이동 |
| 휠 | 확대·축소 (중심점 쪽으로 다가감) |
| `F` | 중심점을 TCP(그리퍼 끝)로 |
| `T` | TCP 따라가기 켜기/끄기 — 픽 동작을 가까이서 볼 때 |
| `1`–`6` | 구도 프리셋: 1 Isaac 롱샷(시작 구도, 책상·보드·계란판 전체), 2 Isaac 녹화 근접, 3 Isaac 측면, 4·5 Isaac 짝 구도, 6 로봇 근접 |
| `R` | 시작 구도로, `H` 카메라 도움말 숨기기 |
| `G` | Isaac HUD(왼쪽 위 패널·왼쪽 아래 그리퍼 카메라 창) 켜기/끄기 |
| `I` | 진단 패널(오른쪽 위: 머리줄 ROS 연결·모드, 타일 수신/발행 Hz·추종 오차, 한계 위반·근처, 수확 수(계란판·바닥)·잡는 중, 관절각) 켜기/끄기. 기본은 Mirror 켜짐, Command 꺼짐 |
| `P` | TCP 궤적(하늘색 선) 켜기/끄기. 기본 꺼짐, `C` 로 지움 |

키 입력은 Game 뷰를 한 번 클릭해 포커스를 준 뒤에 먹는다. 화각은 Isaac 처럼 가로를 고정하므로 Game 창 비율이 달라도 좌우로 보이는 범위는 같다(위아래만 늘거나 준다). **Game 뷰 위쪽의 `Scale` 슬라이더는 카메라 확대가 아니라 픽셀 확대**라서 올리면 화면이 깨진다 — 1x 로 두고 휠로 확대한다. 해상도는 Game 뷰 왼쪽 위 드롭다운에서 `Full HD (1920x1080)` 또는 `Free Aspect` 를 고른다.

### H3-1. Mirror 모드(M1) — Isaac 런의 실제 수확 시퀀스를 그대로 재생

Isaac 에서 녹화한 런(`~/bags/run1`, H4)을 재생한다. 팔·그리퍼는 Isaac 이 실제로 움직인 값 그대로이고, bag 에 담긴 `/sim/grasp_event` 로 잡은 딸기가 그리퍼를 따라간다(7번 잡고 7번 놓는다). bag 앞쪽 46.7 s 는 팔이 서 있는 대기 구간이라 `run_m1.sh` 가 44 s 부터 튼다 — 재생 시작 약 3 s 뒤 팔이 움직이고 첫 잡기는 약 25 s 뒤. 끝까지 약 5분, 끝나면 다시 돈다(딸기도 제자리로 돌아온다).

**이 확인에는 왼쪽 위 HUD 패널이 없다.** HUD 는 딸기 노드 계측에서 나오는데 M1 은 노드 없이 bag 만 튼다. 노드 램프·트리·보드 하이라이트는 H3-2 에서 본다.

터미널 두 개를 쓴다. 순서는 상관없다(에디터는 엔드포인트가 뜰 때까지 1초마다 다시 연결한다).

1. 터미널 A — 엔드포인트와 bag 재생. 끝낼 때 Ctrl-C 한 번이면 둘 다 꺼진다.
   ```bash
   bash scripts/run_m1.sh bag ~/bags/run1
   ```
2. 터미널 B — 에디터를 Mirror 모드로 띄운다. 창이 뜨고 씬이 열린 뒤 ▶ Play 가 자동으로 눌린다(처음엔 1–2분).
   ```bash
   bash scripts/open_editor.sh mirror
   ```
3. Game 화면에서 확인할 것
   - 오른쪽 위 진단 패널(`I`): 머리줄 `ROS 2 연결됨`(녹색)·`M1 · 미러`, 타일 `수신 ≈ 55 Hz`(Isaac 발행 주기), 작은 타일 `계란판`·`바닥` 수와 그 아래 `잡는 중 ripe_xx`.
   - 첫 과실(보드 맨 왼쪽)은 줄기가 그리퍼 중심에서 약 5 mm 옆으로 물린다 — 실기 플래너의 맨 왼쪽 과실 보정(+5 mm)이고 Isaac 에서도 같다(README 한계).
   - 시작은 Isaac 롱샷(1번): 책상·받침판·계란판·바닥·4분면 보드 전체, 보드에 익은 딸기 8개(빨강)·안 익은 딸기 4개(초록).
   - 팔이 분면을 스캔한 뒤 딸기에 다가가 그리퍼를 닫으면, 그 딸기가 그리퍼에 붙어 같이 움직인다. 놓으면 그 자리에서 아래로 떨어진다(계란판 위면 컵 안으로, 아니면 바닥으로 — Isaac 런에서 놓은 자리 그대로).
   - 그리퍼 손가락은 다 닫혀도(61.88°) 서로 겹치지 않는다(간격 0.29 mm).
   - 왼쪽 아래 그리퍼 카메라 창에 손목 D455 시점이 보인다. 왼쪽 위 HUD 패널은 **없다** — HUD 데이터는 딸기 노드 계측에서 오는데 M1 은 노드 없이 bag 만 튼다. 패널·보드 하이라이트는 H3-2 에서 본다.
4. 끝내기: 에디터 창 X(저장하겠냐고 물으면 **Don't Save**) → 터미널 A 에서 Ctrl-C.
5. 이상하면 화면을 캡처해 두고 원인을 본다. 특히 링크 메쉬가 관절에서 떨어져 보이면 시각 축 판정을 다시 본다.

가짜 퍼블리셔(`bash scripts/run_m1.sh fake`)는 한계 감시 시험용이다 — 관절마다 위상을 달리한 사인파로 한계 근처를 훑는다(그리퍼 네 관절은 한 위상으로 같이 움직인다, 8.5–54.5°). 실제 시퀀스가 아니므로 수확 동작 확인에는 bag 을 쓴다. bag 이 없으면 터미널 A 에서 `bash scripts/run_m1.sh fake` 를 띄우고 2번부터 같은 순서로 진행한다 — 로봇이 사인파를 따라 움직이고, 진단 패널·카메라 키·그리퍼 카메라 창은 그대로 볼 수 있다(잡기·HUD 패널은 없다).

### H3-2. Command 모드(M2) — 화면으로 M2 루프 보기 (선택, 30초 영상 후보)

Unity 가 플랜트가 되어 딸기 노드의 명령을 따라 움직이는 것을 화면으로 본다. 노드 기동·트리거는 스크립트가 한다.

1. 터미널 B — 에디터를 Command 모드로 띄운다(▶ Play 자동). **먼저 띄우는 것을 권한다.**
   ```bash
   bash scripts/open_editor.sh command
   ```
   진단 패널(`I` 로 켬)에 `M2 · 명령 루프`, 타일 `상태 발행 … Hz`(/dsr01/joint_states), 아래 줄 `발행 과실 8/8` 이 보이면 준비된 것이다(엔드포인트가 아직 없으면 머리줄이 `ROS 2 연결 오류`·빨강, 엔드포인트가 뜨면 `ROS 2 연결됨`·녹색).
2. 터미널 A — 엔드포인트·딸기 노드·트리거·요약을 스크립트가 한다.
   ```bash
   bash scripts/run_m2_headless.sh --plant editor
   ```
   스크립트는 엔드포인트를 띄운 뒤, **에디터(플랜트)가 overview 자세로 `/dsr01/joint_states` 를 낼 때까지 최대 10분 기다린다**. 이 대기는 '10분 안에 에디터를 켜서 Play 하라'는 뜻이다. 그 안에 상태가 안 오면 `!! 플랜트 상태가 overview 로 오지 않는다` 를 출력하고 아무것도 시작하지 않은 채 정리하고 끝난다(요약은 전부 0·None — 고장이 아니다). 1번처럼 에디터를 먼저 Play 해 두면 바로 넘어간다.
3. 상태를 확인하면 딸기 노드 4개를 띄우고(약 15초, 기동 로그 대조) 트리거한다. 약 5분 동안 팔이 분면을 스캔하고 과실 8개에 차례로 접근·닫기·후퇴·배치 동작을 한다. 잡은 딸기는 그리퍼를 따라가 계란판에 떨어진다. 왼쪽 위에 Isaac 과 같은 HUD 패널(노드 램프·트리·단계·결과 막대)이 뜨고, 보드에는 작업 중인 칸에 청록 테두리와 방위 글자가 켜진다. 왼쪽 아래 그리퍼 카메라 창 오른쪽 위에는 보드 미니맵이 나온다(작업 칸에 있을 때만).
4. 터미널 A 가 `종료 사유: scan_complete` 와 요약을 출력하면 끝이다. 에디터 창 X(**Don't Save**). 결과는 `logs/m2/<시각>_m2_editor/summary.txt`.
5. 한 번 더 돌리려면 **▶ 를 눌러 Play 를 멈췄다가 다시 Play** 한 뒤 터미널 A 를 실행한다. 같은 Play 로 이어 돌리면 앞 런에서 잡거나 수확한 과실이 `/isaac_sim/strawberries` 에서 빠진 채로 남아 overview 스캔이 익은 과실 0개를 보고 3초 만에 `SCAN_COMPLETE` 로 끝난다(`logs/m2/20261022_003302_m2_editor`). 요약은 `픽 시도·종료: False` → 'M2 완료 아니오' 로 판정한다.

## H4 — Isaac 런 한 번 실행하며 bag 녹화

목적: M1 의 `run_m1.sh bag` 재생 입력. 딸기 노드는 **이 리포의 스크립트**로 띄운다(딸기 리포 안에 run_logs·__pycache__·JSONL 을 만들지 않기 위해, CONTRACT §9-4 D-5).
스캔 속도는 기본 60/90(D-7) — Unity M2 런과 같은 조건이다.

터미널 3개.

1. 터미널 1 — Isaac Sim. `PYTHONDONTWRITEBYTECODE=1` 은 Kit 확장 폴더에 `__pycache__` 가 생기지 않게 한다.
   ```bash
   PYTHONDONTWRITEBYTECODE=1 bash ~/strawberry_grasp_environment/scripts/run_isaacsim.sh
   ```
   뜬 뒤 GUI 에서(딸기 리포 docs/run_guide.md 와 같다): 씬 로드 → Script Editor 에서 `strawberry_harvest/scripts/isaac_sim_run_all.py` Run → 콘솔에 `[run_all] done -- press Play` → **Play**.
2. 터미널 2 — 녹화 시작 (CONTRACT §7-5 토픽 그대로)
   ```bash
   source /opt/ros/humble/setup.bash && mkdir -p ~/bags && ros2 bag record -o ~/bags/run1 /dsr01/joint_states /joint_command /isaac_sim/strawberries /sim/grasp_event /strawberry/detection/pick_pose /strawberry/detection/scene_positions /strawberry/exploration/set_cell_state /strawberry/scan/status /dsr01/curobo/pick_pose /dsr01/curobo/pick_complete /dsr01/gripper/position_cmd
   ```
3. 터미널 3 — 딸기 노드(이 리포 스크립트). 기동 로그 대조가 `전부 정합` 이면 같은 터미널 안내대로 트리거한다.
   ```bash
   bash scripts/run_strawberry_nodes.sh
   ```
   ```bash
   source /opt/ros/humble/setup.bash && ros2 service call /strawberry/scan/start std_srvs/srv/Trigger
   ```
4. 녹화 중 한 번, 다른 터미널에서 Isaac 의 벽시계 Hz 를 잰다(리포에 기록이 없는 값):
   ```bash
   source /opt/ros/humble/setup.bash && ros2 topic hz /dsr01/joint_states
   ```
5. 스캔이 끝나면(`scan.log` 에 `SCAN_COMPLETE`) 터미널 2 Ctrl-C(녹화 종료) → 터미널 3 Ctrl-C(노드 정리, 끝에 딸기 리포 변화 여부를 출력) → Isaac Stop·종료.
6. 확인할 것: `~/bags/run1` 이 생겼는지, 4번의 Hz 값, 터미널 3 마지막 줄의 '딸기 리포 상태 변화 없음' 여부. 그다음 `run_m1.sh bag` 재생 확인은 스크립트로 한다.

## H5 — 30초 영상 (단계 8)

헤드리스 batchmode(-nographics)는 화면을 그리지 않아 영상은 에디터에서 찍는다. H3-2(Command 모드, `--plant editor`)를 돌리면서 녹화한다. 터미널 두 개(A: 런 스크립트, B: 에디터)를 쓴다.

**리눅스 에디터의 Recorder 는 MP4(H.264)로 녹화하지 못한다.** Recorder 4.0.3 소스(`CoreEncoderSettings.IsCodecSupportedOnThisPlatform`)가 리눅스에서 MP4 를 막고 기본을 WebM 으로 둔다(공식 문서도 같다). 그래서 **WebM(VP8)으로 녹화하고, MP4 변환·30초 자르기는 녹화 뒤 ffmpeg(libx264)로 한다.**

1. 에디터를 Play 없이 띄운다: `bash scripts/open_editor.sh command --no-play`
2. Recorder(`com.unity.recorder` 4.0.3, 2022.3 에디터 내장)는 이미 `Packages/manifest.json` 에 들어 있다 — 설치할 것 없음. 메뉴 Window → General → Recorder 가 보이면 된다.
3. Window → General → Recorder → Recorder Window → `+ Add Recorder` → Movie.
   - Source `Game View`, Output Resolution `FHD - 1080p`(1920×1080)
   - Format **`WebM`**, **Encoding quality `Custom` → Target Bitrate `20`(Mbps)**, Keyframe Distance 25 그대로, `Include audio` 끔.
     `High` 는 비트레이트를 넘기지 않는 옛 경로라(Recorder 4.0.3 `CoreEncoder.OpenStream`: `Custom` 일 때만 `targetBitRate` 전달) 이 PC 에서 1080p 가 약 250 kbps 로 나와 글자가 뭉개졌다(10-22 시험 녹화 Movie_001·002). `Custom` 20 Mbps 로 바꾼 Movie_003 은 선명했다(정지 화면이라 실측 약 1.4 Mbps).
   - Game 뷰 해상도를 미리 `1920x1080` 으로(없으면 `+` → Fixed Resolution 1920×1080). 안 맞추면 Recorder 가 START 때 Game 뷰 크기를 바꾼다(노란 경고).
   - 출력 경로·파일 이름은 기본값 그대로(`UnityProject/Recordings/`)
   - **`Exit Play Mode` 체크 끔**(Recording Mode 바로 위, 기본 켜짐). 켜져 있으면 `STOP RECORDING` 때 Play 가 끝나 플랜트가 끊기고 런이 완주하지 못한다(Recorder 4.0.3 `RecorderWindow`: `ExitPlayMode` 면 `EditorApplication.isPlaying = false`, 10-22 녹화 런 `20261022_091701_m2_editor` 가 이렇게 끊김).
   - Recording Mode `Manual`, **Playback `Variable`**(WebM 인코더가 받는다 — Variable 을 막는 건 ProRes 뿐). Unity 는 플랜트라 가상 제어기의 벽시계에 맞춰 돌아야 한다. Constant 는 게임 시간을 녹화 프레임에 묶어서, 렌더가 목표 FPS 를 못 따라가면 팔이 실제 시간보다 느리게 움직이고 상태 발행 주기가 바뀐다.
   - Recorder 창은 Game 뷰를 가리지 않게 따로 띄우거나 다른 칸에 도킹한다. Game View 입력은 Game 뷰가 그린 화면을 복사하므로 Game 뷰 탭이 보여야 한다. `START RECORDING` 은 Play 중에 누른다(Play 가 아닐 때 누르면 Recorder 가 Play 를 새로 시작한다).
4. **시험 녹화(약 10초)**: ▶ Play → Game 뷰 클릭 → `I` 로 진단 패널 켬 → Recorder `START RECORDING` → 10초 뒤 `STOP RECORDING` → ▶ 로 Play 멈춤. 녹화 파일에서 프레임을 뽑아 오른쪽 위 진단 패널(OnGUI)이 영상에 찍혔는지 본다. Recorder 의 Game View 입력은 화면 최종 프레임을 복사(`ScreenCapture.CaptureScreenshotIntoRenderTexture`)하므로 HUD 도 찍힌다 — 10-22 시험 녹화에서 확인(진단 패널·왼쪽 아래 그리퍼 카메라 창 모두).
5. **본 녹화**: ▶ Play 를 새로 누른다(H3-2 5번) → 터미널 A `bash scripts/run_m2_headless.sh --plant editor` → Game 뷰 클릭 → `I`(진단 패널 켬)·`H`(오른쪽 아래 카메라 키 안내 숨김 — 1080p 에서 너무 작아 못 읽는다) → 트리거 로그(`Trigger_Response(success=True …)`)가 뜨면 `START RECORDING`. 00:27 런 기준으로 트리거 뒤 5.5 s 에 첫 스캔 자세, 13 s 에 첫 픽 시작, 35 s 에 첫 픽이 끝난다 → **약 45초 녹화 후 `STOP RECORDING`**(30초는 ffmpeg 로 잘라 낸다). 나머지 런은 끝까지 두어 요약을 남긴다.
6. 녹화한 런의 `logs/m2/<시각>_m2_editor/summary.txt` 에서 `판정: M2 완료 예`, `/dsr01/joint_states` 약 60 Hz·'0.3 s 넘는 간격 0' 인지 본다(녹화·MSAA·그림자 부하로 느려지지 않았는지). 끝나면 에디터 창 X(**Don't Save**).
