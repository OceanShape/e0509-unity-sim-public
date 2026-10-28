# e0509 Unity 시뮬레이터

두산 협동로봇 e0509 를 Unity 에 구성하고, Isaac Sim 기반 딸기 수확 프로젝트(이전 프로젝트, 비공개 리포 — 이하 딸기 리포)의 ROS 2 제어 스택을 연결한 두 번째 시뮬레이터. 제어 스택 쪽 파일은 바꾸지 않고 Isaac 자리에 Unity 를 끼워, 같은 스캔·픽(과실 탐색·집기) 시퀀스를 구동한다.

https://github.com/user-attachments/assets/434eb4d9-ba1d-4dd2-9359-bc36c58b0ad5

- Isaac 없이 딸기 리포의 노드 4개가 Unity 속 로봇을 움직이는 장면(30초)
- 동작: 내려가기 → 잡기 → 당기기 → 계란판에 놓기 → 복귀
- 화면: 왼쪽은 작업 상태 HUD 와 그리퍼 카메라, 오른쪽 위는 Unity 진단 패널

## 주요 작업

| 항목 | 내용 | 근거 |
|---|---|---|
| URDF → ArticulationBody | 딸기 리포의 로봇 모델(URDF)을 Unity 로 가져와 관절 단위로 구동되는 로봇 팔을 구성. 관절 구동 설정은 Isaac 과 같게 맞춤 | 기준 자세에서 계산한 끝단 높이 1.123 m 가 URDF 와 일치 |
| ROS 2 연동 (ROS-TCP) | `/joint_command` → 드라이브 목표, 측정 관절값 → `/dsr01/joint_states` 60 Hz, 과실 좌표 → `/isaac_sim/strawberries`. 주고받는 토픽은 전부 표준 메시지 타입 | [docs/CONTRACT.md](docs/CONTRACT.md) |
| M1 — 관절 상태 미러 | Isaac 런을 녹화한 bag(344.6 s)을 받아 Unity 로봇으로 재생 | 300 s 관찰에서 54 Hz 수신, 수신값과 측정값의 차이 10축 모두 0.000° |
| M2 — 루프 완결 (Isaac 없이) | 플래너 → 가상 제어기 → Unity → 상태 회신의 루프로, 픽 8회짜리 스캔·픽 시퀀스를 **11회** 완주 | 런당 274–320 s, 가상 제어기 도착 판정 74–84회·정체 0·타임아웃 0, 관절 한계 위반 0 — [런별 판정](docs/DETAILS.md#m2-런-판정) |
| 관절 한계 감시 | 위치·속도를 URDF 한계값과 여유 없이 비교, 한계 0.5° 이내 접근은 위반과 구분해 기록 | `logs/limit_*.csv` |

작업 기간은 2026-10-20 – 10-22 이며, Unity 는 이 프로젝트에서 처음 사용했다. 딸기 리포의 파일은 바꾸지 않았다(런마다 git 상태를 비교해 확인).

## 구조

```mermaid
flowchart LR
  N[딸기 리포 노드] <-->|ROS 2| E[ROS-TCP 브리지] <-->|TCP| U[Unity 시뮬레이터]
```

- 딸기 리포 노드 4개: 플래너 · 스캔 실행기 · 비전 모킹 · 가상 두산 제어기
- 노드 → Unity: 관절 명령 `/joint_command`
- Unity → 노드: 관절 상태 `/dsr01/joint_states`(60 Hz), 과실 좌표 `/isaac_sim/strawberries`

파지 판정은 Unity 가 아니라 가상 제어기가 한다. Unity 가 명령값을 그대로 돌려보내면 실패를 감지할 수 없으므로, Command 모드는 드라이브 목표만 주고 물리 스텝 뒤의 측정값을 회신한다.

## 실행

이 리포만으로 돌려 볼 수 있는 것은 Unity 단독 시험과 가짜 퍼블리셔 미러까지다. M2 와 bag 미러는 리포에 없는 자원이 필요하다.

| 실행 | 필요한 것 | 이 리포만으로 |
|---|---|---|
| Unity 단독 (ROS 없이) | Unity 2022.3 | 가능 |
| M1 — 가짜 퍼블리셔 미러 | + ROS 2 Humble, ROS-TCP-Endpoint | 가능 |
| M1 — Isaac 런 bag 미러 | + Isaac 런을 녹화한 bag | 불가 — bag 은 리포에 없다 |
| M2 — 루프 완결 | + 딸기 리포 워크스페이스 빌드, cuRobo 가 돌 NVIDIA GPU | 불가 — 딸기 리포는 비공개다 |

### 준비 (처음 한 번)

Ubuntu 22.04, ROS 2 Humble, Unity Hub 로 설치한 Unity 2022.3 에디터(기본 위치 `~/Unity/Hub/Editor/2022.3.*`). 작업은 2022.3.62f2 로 했고 다른 패치 버전은 시험하지 않았다. Unity 는 화면 없는 batch 실행에도 라이선스가 필요하므로 Hub 에 로그인해 Personal 라이선스를 활성화해 둔다. batch 스크립트는 에디터가 이 프로젝트를 열고 있으면 멈추므로 에디터를 닫고 실행한다.

```bash
bash scripts/check_env.sh   # Unity 에디터 경로를 scripts/env.sh 에 저장. FAIL 중 '딸기 ws'·'nvidia-smi' 는 M2 에만 해당한다
git clone -b ROS2v0.7.0 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros/src/ROS-TCP-Endpoint
source /opt/ros/humble/setup.bash && (cd ros && colcon build)
bash scripts/unity_batch.sh 03 configure -executeMethod SimSetup.ConfigureProject
bash scripts/unity_batch.sh 04 build_scene -executeMethod SimSetup.BuildScene
```

에디터가 다른 곳에 있으면 `UNITY_BIN=/경로/Editor/Unity bash scripts/check_env.sh`. 처음 프로젝트를 열 때 Unity 패키지(URDF-Importer·ROS-TCP-Connector 는 GitHub 에서 git 으로, Recorder 등은 Unity 레지스트리에서)를 받으므로 네트워크가 필요하다. 작업 PC 에서 패키지 캐시를 비우고(`UPM_CACHE_ROOT` 를 빈 폴더로) 새로 클론해 잰 첫 configure 는 18 s(그중 패키지 받기·해석 4.6 s, 약 90 MB)였고, build_scene 약 10 s, 아래 PlayMode 시험은 각각 30 s 안이다.

### Unity 단독 (ROS 없이)

```bash
bash scripts/unity_test_command.sh   # Command 모드 추종 시험(PlayMode 테스트 CommandModeTest, 에디터는 닫은 상태). 통과면 출력에 result="Passed", 결과 logs/m2/<시각>_command_mode/
bash scripts/open_editor.sh command  # 에디터에서 씬 E0509Sim 을 열고 Play. 명령이 없으니 로봇은 시작 자세로 서 있다
```

CommandModeTest 는 ROS 콜백과 같은 경로로 관절 명령을 넣고, 자세 유지·한계 직전 계단 명령·가상 제어기와 같은 사다리꼴 궤적·그리퍼 닫힘에서 0.3° 안 도착과 한계 위반 0 을 확인한다. 단계별 수치는 결과 폴더의 `command_mode_report.json`. 에디터 화면에서는 Game 뷰를 클릭한 뒤 `I` 로 진단 패널, `1`–`6` 으로 구도, 오른쪽 드래그·휠로 카메라를 움직인다([docs/HUMAN_STEPS.md 화면 조작](docs/HUMAN_STEPS.md#화면-조작-game-뷰-play-중)). 에디터 Console 의 빨간 줄 두 종류는 정상이다: Play 때 한 번 나오는 `DllNotFoundException: libdl.so`(URDF-Importer 의 Assimp 플러그인 초기화, 씬 생성과 시험 결과에 영향 없음)와, 엔드포인트 없이 띄웠을 때 1초마다 나오는 ROS 연결 실패(엔드포인트가 뜨면 붙는다). 에디터는 창의 X 로 닫는다 — 터미널에서 `kill` 로 끄면 Unity 가 크래시로 처리해 Unity Bug Reporter 창이 뜬다(보내지 말고 Cancel).

### M1 — 가짜 퍼블리셔 미러

ROS 를 거쳐 로봇이 움직이는 것을 보는 가장 짧은 길이다.

```bash
bash scripts/run_m1.sh fake         # 엔드포인트 + 가짜 /dsr01/joint_states(60 Hz, 관절마다 사인파). Ctrl-C 로 끈다
bash scripts/open_editor.sh mirror  # 다른 터미널에서: 에디터가 열리고 Play 까지 자동, 로봇이 사인파를 따라 움직인다
bash scripts/unity_test_m1.sh       # 화면 없이 같은 미러를 PlayMode 테스트로(수신 주기·명령-측정 차이·한계 위반 감지, 에디터는 닫은 상태). 통과면 result="Passed", 결과 logs/m1/<시각>/
```

### 리포에 없는 자원이 필요한 실행

```bash
bash scripts/run_m1.sh bag ~/bags/run1   # M1: Isaac 런 bag 미러 — bag 이 필요하다
bash scripts/run_m2_headless.sh          # M2: 엔드포인트 → Unity → 딸기 노드 → SCAN_COMPLETE — 딸기 리포 워크스페이스(STRAWBERRY_WS, 기본 ~/strawberry_grasp_environment)와 GPU 가 필요하다
```

두 실행의 결과는 커밋된 로그(`logs/m1/`, `logs/m2/`)와 [런별 판정](docs/DETAILS.md#m2-런-판정), 위 영상으로 볼 수 있다.

## 범위

범위는 운동학 구동(드라이브 추종)과 관절 한계 감시까지다. 중력·접촉은 끄고 과실 낙하는 운동학적으로 처리했으며, 동역학 검증은 범위 밖이다. 알려진 한계는 [docs/DETAILS.md](docs/DETAILS.md#한계) 의 '한계' 절.

## 더 보기

- [docs/DETAILS.md](docs/DETAILS.md) — 상세 문서: 토픽 계약, 측정치 전체, 화면, 한계, 출처
- [docs/CONTRACT.md](docs/CONTRACT.md) — 딸기 리포와 주고받는 토픽·역할 분담(경계 계약)
- [docs/HUMAN_STEPS.md](docs/HUMAN_STEPS.md) — 수동 단계: 에디터 화면 조작 키, Isaac 런 bag 녹화, 30초 영상 녹화
- [docs/PROGRESS.md](docs/PROGRESS.md) — 단계별 진행 기록

## 라이선스

코드·문서·직접 만든 자산은 MIT([LICENSE](LICENSE), © 2026 OceanShape). 두산·ROBOTIS 메쉬는 각 원저작자의 라이선스를 따른다 — [THIRD_PARTY.md](THIRD_PARTY.md).
