# CONTRACT — 딸기 리포의 플랜트 경계 계약 (Isaac → Unity)

> **문서 종류**: 단계 1 정찰 결과. 딸기 리포를 읽기만 해서 만든 계약서다.
> **기준**: 딸기 리포 `~/strawberry_grasp_environment` HEAD `2797bb5` (2026-10-15), 조사일 2026-10-20. 딸기 리포는 비공개라 아래 `파일:줄` 근거는 이 리포 밖에서 따라갈 수 없다. 계약의 내용(토픽·타입·자세·규칙) 자체는 이 문서만으로 읽힌다.
> **조사 방법**: 다섯 갈래(URDF·관절 / Isaac 쪽 / 가상 제어기 / 플래너·스캔·비전 / 환경·메시지·시간)로 나눠 정찰했다. 갈래마다 독립 검증자가 코드 줄과 다시 대조했고, 리포 전체의 ROS 이름을 따로 전수 조사해 빠진 것을 찾았다. 결론을 좌우하는 주장(관절 상태 읽는 방식, 파지 판정 경로, 도착 판정, Isaac 발행 코드, 로그 경로)은 작성자가 코드에서 직접 다시 확인했다.
> **딸기 리포 변경**: 이번 조사에서 딸기 리포 파일 변경 없음. `git status` 는 조사 전과 같다(조사 전부터 있던 미추적 mp4 4개뿐).

**근거 표기.** `파일:줄` 은 딸기 리포 기준 상대 경로다. 자주 쓰는 파일은 줄여 쓴다.

| 약어 | 파일 |
|---|---|
| `I` | `strawberry_harvest/scripts/isaac_sim_script_editor_bridge.py` (Isaac 안에서 도는 rclpy 브릿지) |
| `B` | `src/strawberry_sim_core/strawberry_sim_core/sim_executor_bridge_node.py` (가상 두산 제어기) |
| `CPN` | `src/strawberry_motion/scripts/curobo_planner_node.py` (플래너) |
| `SE` | `src/strawberry_motion/strawberry_motion/execution/scan_executor_node.py` (스캔 실행기) |
| `FV` | `src/strawberry_sim_core/strawberry_sim_core/fake_vision_node.py` (비전 모킹) |
| `R` | `scripts/run_nodes.sh` (노드 4개 기동) |

줄 번호는 `src/` 기준이다. 실제로 `ros2 run` 되는 것은 `install/` 의 복사본(심볼릭 링크 아님)인데, 주석·독스트링만 다르고 코드(AST)는 같다. 그래서 동작은 같고 줄 번호만 최대 27줄 어긋난다.

---

## 0. 한눈에

### 0-1. 가장 중요한 사실 8가지

1. **Isaac 이 ROS 와 주고받는 것은 토픽 4개뿐이다.** 서비스·액션·`/clock`·TF 는 없다. 전부 표준 메시지 타입이다(§3).

   | 방향 | 토픽 | 타입 |
   |---|---|---|
   | Isaac → 노드 | `/dsr01/joint_states` | `sensor_msgs/JointState` |
   | Isaac → 노드 | `/isaac_sim/strawberries` | `geometry_msgs/PoseArray` |
   | 노드 → Isaac | `/joint_command` | `sensor_msgs/JointState` |
   | 노드 → Isaac | `/sim/grasp_event` | `std_msgs/String` |

2. **상태 토픽 이름은 `/joint_states` 가 아니라 `/dsr01/joint_states` 다**(`I:65`). 처음 계획의 JointStateMirror 기본 토픽과 H4 녹화 명령을 이 이름으로 고쳐야 한다(§9).
3. **파지 판정은 Isaac 이 하지 않는다.** 가상 제어기가 `/dsr01/joint_states` 앞 6개를 cuRobo FK 에 넣어 TCP 를 구하고, `/isaac_sim/strawberries` 과실 좌표와의 거리로 판정한다(`B:545-616`). 그래서 Unity 는 파지 판정을 재현할 필요가 없다. **측정한 관절값과 과실 좌표만 정확히 내면 된다.** 처음 계획 단계 7의 "판정 토픽 bag 재생 / 판정 없는 구간까지만" 선택은 필요 없어진다.
4. **가상 제어기는 관절 상태를 이름이 아니라 순서로 읽는다.** `position[:6]` 을 J1–J6 로 쓴다(`B:515-518`). Unity 가 내는 `/dsr01/joint_states` 는 앞 6칸이 `joint_1..joint_6` 이어야 한다. 플래너·스캔은 이름으로 찾는다.
5. **도착 판정 결과는 서비스 응답에 실리지 않는다.** MoveJoint·MoveSplineJoint 는 도착하든 못 하든 `success=True` 다. 결과는 브릿지 로그(`ARM_ARRIVAL` / `ARM_ARRIVAL_STALLED` / `ARM_ARRIVAL_TIMEOUT`)에만 남는다(`B:681-743,805-809,876-877`). M2 완료 기준은 이 로그 줄로 판정해야 한다.
6. **관절 상태가 한 번도 안 오면 가상 제어기는 조용히 넘어간다.** 도착 대기를 0.2초 만에 `True` 로 끝낸다(`B:697-699`). 반대로 과실 좌표가 없으면 현행 닫기 경로는 빈손(`GRASP_EMPTY`)으로 떨어진다(`B:1114-1119`). "항상 성공"은 꺼져 있는 SafeGrasp 경로에만 남아 있다(`B:1146-1148`).
7. **시간은 전부 벽시계다.** `use_sim_time`·`/clock` 을 쓰는 곳이 없다. Unity 는 `/clock` 을 내지 않고 노드도 기본값(false)으로 둔다(§4).
8. **Isaac 런 bag 은 아직 없다.** `~/bags` 는 비어 있고 리포에도 rosbag 이 없다. M1 의 bag 재생은 H4 녹화가 있어야 한다(§7-5).

### 0-2. 결정이 필요한 항목

자세한 설명은 §9 에 있다. **2026-10-20 에 전부 확정했다. 확정 내용은 §9-4** 이고, 아래 표의 권장 열은 확정 전 제안이다(D-3·D-6·D-7 은 권장과 다르게 확정).

| # | 결정 | 권장 | 필요 시점 |
|---|---|---|---|
| D-1 | Unity URDF 의 원천 | 리포 `robot.urdf` 에서 사본을 만들고 세 군데만 고친다 | 단계 3 |
| D-2 | 그리퍼 `rh_r2`·`rh_l2` 상한 | Isaac 과 같은 1.1 rad | 단계 3 |
| D-3 | 관절 한계 판정 여유(엡실론) | 0.01° 정도의 작은 여유 | 단계 5 |
| D-4 | M2 Command 모드의 구동 방식 | 드라이브 목표만 주고 측정값을 회신. `jointPosition` 직접 대입 금지 | 단계 5·7 |
| D-5 | 딸기 노드 실행 방식(리포에 파일이 생기는 문제) | Unity 시뮬레이터(두 번째 엔진) 리포의 실행 스크립트 + 로그 경로 환경 변수 | H4 전, 단계 7 |
| D-6 | M2 의 과실 좌표 공급원 | Unity 가 정적 배치를 발행하고 부착 과실을 뺀다 | 단계 7 |
| D-7 | 스캔 MoveJoint 속도(시뮬 전용 120/180) | Isaac 기준 런과 같게 120/180 유지, 차이로 기록 | 단계 7 |

---

## 1. URDF 생성 명령

### 1-1. Isaac 로봇 모델이 만들어진 경로

| 단계 | 파일 | 하는 일 | 근거 |
|---|---|---|---|
| ① 최상위 xacro | `src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro` | 팔·그리퍼 xacro 를 **절대 경로**로 include. `arm_to_gripper`(link_6 → rh_p12_rn_base, rpy 0 0 90°)와 커스텀 파츠 좌·우 링크(visual 만, inertial 0.1 kg)를 붙인다 | `robot.urdf.xacro:7,13,20-24,35-84` |
| ② 팔 | `src/doosan_robot2/dsr_description2/xacro/e0509.urdf.xacro` → `macro.e0509.white.xacro` | `world` 링크와 `world_fixed`(world → base_link)를 만든다. gz·mj 가 아니면 기본 ros2_control 블록을 넣는다 | `e0509.urdf.xacro:5-15,30-37,64-75` |
| ③ 그리퍼 | `src/RH-P12-RN/rh_p12_rn_description/urdf/rh_p12_rn.xacro` | **`world` 링크와 `world_fixed`(world → rh_p12_rn_base)가 그대로 남아 있다.** `<mimic>` 은 없다 | `rh_p12_rn.xacro:13-21` |
| ④ 빌드 스크립트 | `build_urdf.sh` | xacro 결과로 **리포 루트 `robot.urdf` 를 덮어쓴다.** `package://` 3종을 절대 경로로, grey 재질을 black 으로 바꾼다 | `build_urdf.sh:11,15-17,21-22` |
| ⑤ 리포 `robot.urdf` | Isaac 용 URDF 로 문서화된 파일 | ④의 결과에 손편집 두 가지가 더해져 있다(§1-4) | `robot.urdf:124-126,170-172,260-264,866,891` |
| ⑥ Isaac 로봇 USD | `strawberry_harvest/assets/robot/doosan_e0509_rh_p12_rn/` | 07-03 임포트본이다. 임포트 원본은 **현재 `robot.urdf` 가 아니라 06-18 파일럿 URDF** 다. 09-14 에 `robot.usd` 에 J2·J3·J5 한계를 덮어썼다 | `git log` 해당 폴더, 커밋 `a2e9bb5` |
| ⑦ 조립본 | `strawberry_harvest/assets/robot/robot_assembly.usd` | 드라이브 게인, `rh_r2`·`rh_l2` 상한 1.1 rad, 커스텀 파츠 6° 제거, D455 카메라를 덮어쓴다 | `robot_assembly.usd:122-251` |

- **Isaac 이 실제로 쓰는 값은 ⑥ + `robot.usd` 덮어쓰기 + ⑦ 의 합성값이다.** 파일 하나로 존재하지 않는다. 관절 값으로는 리포 `robot.urdf` 와 같다(그리퍼 상한만 다르다, §2-3).
- 런타임 ROS 노드 가운데 `robot.urdf` 를 읽는 것은 없다. `robot_state_publisher`·`robot_description` 도 없다. ROS 쪽 기구학은 전부 cuRobo 설정 `src/e0509_gripper_description/config/curobo/e0509_gripper.urdf` 를 쓴다(`B:346-362`).
- 리포 루트의 `robot/` 폴더(07-03 USD)는 현행 씬이 참조하지 않는 옛 백업이다. D1 이전의 관대한 한계(J2 ±360 등)를 갖고 있으니 대조에 쓰면 안 된다(`robot/README.md:1-6`, `docs/parameters.md:254`).
- `src/doosan_robot2`·`src/RH-P12-RN` 은 딸기 리포의 `.gitignore` 대상이고 각각 별도 git 저장소다(doosan `ec92425`, RH-P12-RN `58d06e7`). 로컬 수정은 `$(find …)` 를 이 PC 의 절대 경로로 바꾼 것과, `dsr_description2/config/macro.gazebo.xacro` 의 `dsr_controller2` 파라미터 줄 주석 처리다. 두 번째 수정이 없으면 `dsr_controller2` 가 설치되지 않은 이 PC 에서 렌더가 실패한다.

### 1-2. 그대로의 xacro 변환 명령 (처음 계획의 요구)

출력은 반드시 딸기 리포 밖으로 보낸다. 이 조사에서 실행해 결과를 확인했다.

```bash
source /opt/ros/humble/setup.bash
SRC=$HOME/strawberry_grasp_environment
xacro "$SRC/src/strawberry_grasp_environment_description/urdf/robot.urdf.xacro" \
      color:=white gripper:=none namespace:= use_gazebo:=false use_mujoco:=false \
      > $HOME/e0509_unity_sim/assets_src/robot_xacro_raw.urdf
```

- 인자는 전부 기본값이다. 인자를 빼도 결과는 바이트까지 같다(헤더 주석 경로만 다름).
- include 가 전부 절대 경로라 `ROS_PACKAGE_PATH` 는 필요 없다.
- stderr 에 `redefining global symbol: pi` 경고가 2줄 나온다. 무해하다.
- **`build_urdf.sh` 는 쓰지 않는다.** 리포 `robot.urdf` 를 덮어써 손편집(D1 한계, 파츠 pitch 0)이 사라진다.
- 렌더 결과: 링크 16 · 관절 15(revolute 10) · `world` 링크 2개 · `world_fixed` 2개 · `rh_p12_rn_base` 의 부모 2개 · `<mimic>` 0개. 기구와 무관한 ros2_control·transmission(이름이 `joint1..6` 으로 다름)·gazebo 블록도 들어 있다.

### 1-3. xacro 결과와 리포 `robot.urdf` 의 차이 (전부)

`build_urdf.sh` 의 sed 5개를 xacro 결과에 똑같이 적용하면 아래 #3–#7 만 남는다.

| # | 항목 | xacro 결과 | 리포 `robot.urdf` | 언제·왜 | 근거 |
|---|---|---|---|---|---|
| 1 | 메쉬 경로 40곳 | `package://` | `/home/oceanshape/strawberry_grasp_environment/src/…` | `build_urdf.sh` sed. Isaac Importer 용 | `build_urdf.sh:15-17` |
| 2 | 그리퍼 재질 참조 | grey | black | `build_urdf.sh` sed(시각만) | `build_urdf.sh:21-22` |
| 3 | joint_2 한계 | ±6.2832 | **±1.6581** (±95°) | 09-14 손편집, 커밋 `a2e9bb5`, D1 | `robot.urdf:124-126`, `docs/e0509_spec_audit.md:104` |
| 4 | joint_3 한계 | ±2.7053 | **±2.3562** (±135°) | 같음 | `robot.urdf:170-172` |
| 5 | joint_5 한계 | ±6.2832 | **±2.3562** (±135°) | 같음 | `robot.urdf:260-264` |
| 6 | `l2_to_left_custom_part` pitch | 6° | 0 | 09-08 손편집, 커밋 `5129c89` | `robot.urdf:866`, `robot.urdf.xacro:57` |
| 7 | `r2_to_right_custom_part` pitch | 6° | 0 | 같음 | `robot.urdf:891`, `robot.urdf.xacro:83` |

**결론: xacro 결과를 그대로 쓰면 J2·J3·J5 한계가 Isaac 보다 넓다.** "시뮬은 원본보다 관대하면 안 된다" 원칙을 지키려면 Unity URDF 는 리포 `robot.urdf` 에서 만들어야 한다(D-1).

### 1-4. Unity 임포트용 사본 (권장안, D-1)

리포 `robot.urdf` 를 Unity 시뮬레이터 리포로 복사한 뒤 사본만 고친다.

1. 절대 경로 3종을 `package://` 로 되돌린다(Unity 배치 규칙: URDF 옆에 패키지명 폴더).
2. **두 번째 `<link name="world">` 와 child 가 `rh_p12_rn_base` 인 `world_fixed` 를 지운다**(`robot.urdf:637-645`). 남기면 그리퍼 베이스의 부모가 둘이라 트리가 깨진다. Isaac USD 에도 이 관절은 없다.
3. `rh_r2`·`rh_l2` 의 upper 를 1.0 에서 1.1 rad 로 올린다(D-2). Isaac 조립본과 같은 값이다(`robot_assembly.usd:185-190,198-203`). 이게 없으면 가상 제어기의 닫힘 명령 1.08 rad 에서 두 관절만 1.0 에 걸려 손가락이 엇갈린다.

결과: 링크 15, 관절 14, 부모가 둘인 링크 0, 루트 `world`. Isaac 의 robotLinks 15·robotJoints 14 와 같다. 선택 사항으로 ros2_control·transmission·gazebo 블록을 사본에서 지우면 임포터 경고가 줄어든다. 변환 스크립트는 단계 3에서 `scripts/` 에 만든다.

**단계 3 결과 (2026-10-20, D-1 확정).** `scripts/make_unity_urdf.py` 로 만들었다. 실행 기록은 `logs/step3/make_unity_urdf.log`.

| 항목 | 값 |
|---|---|
| 원천 파일 | `~/strawberry_grasp_environment/robot.urdf` |
| 딸기 리포 커밋 | HEAD `2797bb535a8a6e02acc6155c7711f43eeec54cdf`(2026-10-15). `robot.urdf` 마지막 변경은 `a2e9bb5`(2026-09-14, D1 한계). 작업 트리 변경 없음 |
| 원천 sha256 | `31518983405e61b26012fbc9ab3472b4c797590a3fe4a0ffbf3af9d471965a01` |
| 메쉬 저장소 | `src/doosan_robot2` @ `ec92425`, `src/RH-P12-RN` @ `58d06e7`(둘 다 메쉬 폴더 변경 없음). `gripper_parts.stl` 은 딸기 리포 추적 파일 |
| 결과 | `assets_src/e0509/e0509.urdf` + 메쉬 34개(12.5 MB, 원본 그대로 복사). 단계 4가 이 폴더를 `UnityProject/Assets/URDF/e0509/` 로 복사한다 |
| 바꾼 것 | 위 1–3 그리고 선택 사항(ros2_control 1·transmission 10·gazebo 15 블록 삭제). 머리 주석은 출처 주석으로 바꿨다 |
| 검증 22건 통과 | 링크 15·관절 14·루트 `world`·부모 둘인 링크 0 / 관절 14개가 원천과 같음(그리퍼 상한 2곳만 다름) / 링크 15개의 관성·visual·collision 이 원천과 같음 / 한계 10개가 §2-4 Isaac 합성값과 같음 / 메쉬 40곳 전부 `package://`, 34개 파일 sha256 이 원본과 같음, 심볼릭 링크 없음 / FK 영점 `rh_p12_rn_base` z 1.123 m, TCP 영점 (0, 0, 1.359)·시작 자세 (0.0226, 0.2151, 0.6426) / xmllint 통과 / 원천 sha256 전후 같음 |

FK 영점 TCP 의 x·y 에 3 µm 가 남는다. 관절 원점 rpy 가 π/2 가 아니라 `1.5708` 로 적혀 있어서다. 원본 기하 그대로이고 Isaac 도 같은 값을 읽는다.

### 1-5. 메쉬

| `package://` 패키지 | 실제 경로(딸기 리포 기준) | 파일 | 형식 | 용도 |
|---|---|---|---|---|
| `dsr_description2` | `src/doosan_robot2/dsr_description2/meshes/e0509_white/` | `E0509_0_0`, `1_1`–`1_4`, `2_1`·`2_2`, `3_1`·`3_2`, `4_1`·`4_2`, `5_1`·`5_2`, `6_1` `.dae` (14) | DAE, `<unit meter="1"/>`, `Z_UP`, URDF scale 0.001(정점은 mm). 파일마다 노드 `<matrix>` 이동이 있고 `E0509_3_2` 는 X 축 90° 회전도 있다 | visual |
| `dsr_description2` | `…/meshes/e0509_collision/` | 같은 이름 14개 `.dae` | 같음. visual 과 거의 같은 고해상도 | collision |
| `rh_p12_rn_description` | `src/RH-P12-RN/rh_p12_rn_description/meshes/` | `base`, `r1`, `r2`, `l1`, `l2` `.stl` | 바이너리 STL, mm(scale 0.001) | visual·collision 겸용 |
| `strawberry_grasp_environment_description` | `src/strawberry_grasp_environment_description/meshes/` | `gripper_parts.stl` | 바이너리 STL, mm | visual 만(좌·우 2곳). collision 없음 |

- 확장자는 `.dae`·`.stl` 뿐이다. 처음 계획 단계 3의 허용 목록 안이다.
- 영점 자세에서 `rh_p12_rn_base` 는 z 1.123 m 다(0.2045 + 0.373 + 0.373 + 0.1725). 임포트 뒤 이 높이와 `E0509_3_2` 위치를 눈으로 확인한다.

---

## 2. 관절 목록

### 2-1. 관절 표 (기구 = 리포 `robot.urdf`)

| 관절 | 타입 | 부모 → 자식 | origin xyz (m) / rpy (rad) | 축 | lower/upper (rad) · vel (rad/s) · effort | mimic |
|---|---|---|---|---|---|---|
| `world_fixed` | fixed | world → base_link | 0 / 0 | — | — | — |
| `joint_1` | revolute | base_link → link_1 | (0, 0, 0.2045) / 0 | z | ±6.2832 · 3.1416 · 194 | 없음 |
| `joint_2` | revolute | link_1 → link_2 | 0 / (0, −1.5708, −1.5708) | z | ±1.6581 · 3.1416 · 194 | 없음 |
| `joint_3` | revolute | link_2 → link_3 | (0.373, 0, 0) / (0, 0, 1.5708) | z | ±2.3562 · 3.1416 · 194 | 없음 |
| `joint_4` | revolute | link_3 → link_4 | (0, −0.373, 0) / (1.5708, 0, 0) | z | ±6.2832 · 3.927 · 66 | 없음 |
| `joint_5` | revolute | link_4 → link_5 | 0 / (−1.5708, 0, 0) | z | ±2.3562 · 3.927 · 66 | 없음 |
| `joint_6` | revolute | link_5 → link_6 | (0, −0.1725, 0) / (1.5708, 0, 0) | z | ±6.2832 · 3.927 · 66 | 없음 |
| `arm_to_gripper` | fixed | link_6 → rh_p12_rn_base | 0 / (0, 0, π/2) | — | — | — |
| `world_fixed` (중복) | fixed | world → rh_p12_rn_base | 0 / 0 | — | — | — (**Unity 사본에서 지움**) |
| `rh_p12_rn` | revolute | rh_p12_rn_base → rh_p12_rn_r1 | (0, 0.008, 0.048) / 0 | +x | 0–1.1 · 6.5 · 1000 | 없음 |
| `rh_r2` | revolute | rh_p12_rn_r1 → rh_p12_rn_r2 | (0, 0.0493634, 0.0285) / 0 | −x | 0–1.0 · 6.5 · 1000 | 없음 |
| `rh_l1` | revolute | rh_p12_rn_base → rh_p12_rn_l1 | (0, −0.008, 0.048) / 0 | −x | 0–1.1 · 6.5 · 1000 | 없음 |
| `rh_l2` | revolute | rh_p12_rn_l1 → rh_p12_rn_l2 | (0, −0.0493634, 0.0285) / 0 | +x | 0–1.0 · 6.5 · 1000 | 없음 |
| `l2_to_left_custom_part` | fixed | rh_p12_rn_l2 → left_custom_part | (0.02090, 0.00498, 0.00058) / (−π/2, 0, −π/2) | — | — | — |
| `r2_to_right_custom_part` | fixed | rh_p12_rn_r2 → right_custom_part | (−0.02090, −0.00499, 0.00058) / (π/2, 0, π/2) | — | — | — |

근거: `robot.urdf:47-54,119-128,165-174,211-218,255-266,303-310,471-477,637-645,668-676,709-717,750-758,791-799,833-837,863-867,888-892`. 그리퍼 네 관절 모두 `<dynamics damping="0.7"/>` 이 있다.

### 2-2. DOF 순서와 이름

| 쓰는 곳 | 순서·이름 | 근거 |
|---|---|---|
| Isaac `/dsr01/joint_states` 의 `name` | `joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, rh_l1, rh_p12_rn, rh_l2, rh_r2` (PhysX DOF 순서, 10개) | `I:610-622`, `log/m3/*/kit_bridge.log:3` (09-10 이후 모든 런 동일) |
| 가상 제어기 `/joint_command` 의 `name` | `joint_1..joint_6, rh_p12_rn, rh_r2, rh_l1, rh_l2` (10개, 매번 전부) | `B:435,647,657` |
| cuRobo·실기 이름 | `gripper_rh_r1/r2/l1/l2`, 링크 `gripper_rh_p12_rn_base` | `e0509_gripper.urdf:351-456` |

두 방향의 순서가 다르다. Isaac 은 `/joint_command` 를 이름으로 매핑하고(`I:221-251`), 가상 제어기는 `/dsr01/joint_states` 를 순서로 읽는다(`B:515-518`). **Unity 는 받을 때 이름으로 매핑하고, 낼 때 Isaac 순서를 그대로 지킨다.**

### 2-3. 그리퍼 네 관절의 연동

| 층 | 방식 | 근거 |
|---|---|---|
| URDF·xacro | `<mimic>` 없음. 네 관절이 독립 | `rh_p12_rn.xacro:49-194` |
| Isaac USD | mimic·tendon 없음. 관절마다 각도 드라이브 | `robot_physics.usd` 덤프 |
| 가상 제어기 | 네 관절에 **같은 각** `clamp(stroke × 1.08 / 700, 0, 1.08)` rad 를 싣는다. stroke 600 → 0.926, 700 → 1.08 | `B:265,479-490,657-658` |
| 축 부호 | (+x, −x, −x, +x) 라 같은 양의 각에서 두 손가락이 안쪽으로 닫힌다 | 위 관절 표 |

`/dsr01/joint_states` 의 그리퍼 네 값은 어떤 노드도 읽지 않는다(가상 제어기는 `[:6]`, 플래너·스캔은 `joint_1..6` 이름만). 그래서 그리퍼 측정값은 계약상 필수가 아니다. 다만 Isaac 과 같은 모양을 내려면 함께 낸다.

### 2-4. 한계 출처 비교 (deg)

| 관절 | xacro 공식 | 리포 robot.urdf | Isaac 합성값 | 가상 제어기 차단선 | cuRobo URDF | 플래너 운용 한계 |
|---|---|---|---|---|---|---|
| J1 | ±360 | ±360 | ±360 | 360 | ±360 | ±225 |
| J2 | ±360 | ±95 | ±95 | 95 | ±95 | ±95 |
| J3 | ±155 | ±135 | ±135 | 135 | ±135 | ±135 |
| J4 | ±360 | ±360 | ±360 | 360 | ±360 | ±360 |
| J5 | ±360 | ±135 | ±135 | 135 | ±135 | ±130 |
| J6 | ±360 | ±360 | ±360 | 360 | ±360 | ±225 |
| 그리퍼 r1·l1 | 0–63.03 | 0–63.03 | 0–63.03 | 명령 0–61.88 | 0–63.08 | — |
| 그리퍼 r2·l2 | 0–57.30 | 0–57.30 | **0–63.03** (조립본 덮어쓰기) | 명령 0–61.88 | mimic | — |

근거: Isaac 합성값 = `robot_physics.usd`·`robot.usd` 덤프 + `robot_assembly.usd:185-203`. 가상 제어기 = `B:433`(판정 `abs(v) > lim`, `B:638`). cuRobo = `e0509_gripper.urdf:48,120,164,208,252,296`. 플래너 = `harvest_motion_params.py:159-166`. 결정 기록 = `docs/e0509_spec_audit.md:104-106`.

속도 한계: J1–J3 180 °/s, J4–J6 225 °/s, 그리퍼 372.4 °/s. xacro·URDF·USD·cuRobo 가 모두 같다. 가상 제어기 실행 속도는 별도로 MoveSplineJoint 가 120 °/s 이하로 페이싱한다(`B:137`).

### 2-5. JointLimitMonitor 기준 (제안)

1. **위치 하드 한계 = J1 ±360, J2 ±95, J3 ±135, J4 ±360, J5 ±135, J6 ±360 (deg). 판정은 `|q| > 한계`.** Isaac 합성값, 가상 제어기 차단선, cuRobo 계획 모델 세 곳이 같다. xacro 공식값은 원본보다 관대해서 쓰지 않는다.
2. **운용 한계(J1 ±225, J5 ±130, J6 ±225)는 하드 한계로 쓰지 않는다.** CSV 에 '운용 한계 경고' 열로 따로 남긴다. 하드로 쓰면 Isaac 이 실행한 궤적을 Unity 만 위반으로 잡는다.
3. **여유가 0 이다.** 정상 시퀀스가 J3 135.0° 까지 쓰고, 시작 자세 J2 −94.92° 는 한계까지 0.08° 남는다. 읽기 잡음으로 거짓 위반이 날 수 있어 작은 여유가 필요하다(D-3). → **확정: 여유 없이 센다. 한계 근처 접근은 따로 기록한다(§9-4).**
4. **그리퍼**: 0–1.1 rad(63.03°). 1.0 rad 초과는 '실기 대비 의도한 시뮬 편차' 정보로만 남긴다(`docs/parameters.md:157-164`).
5. **속도**: 위 속도 한계. Isaac 의 `/dsr01/joint_states` 에는 velocity 가 없고 `/joint_command` 는 계단형이라, 속도는 ArticulationBody 의 실측 관절 속도로 잰다.

### 2-6. TCP

| 주체 | 정의 | 근거 |
|---|---|---|
| 플래너(시뮬) | ee 링크 `gripper_rh_p12_rn_base`, TCP = ee + R·(0, 0, **0.236**) m. 0.236 은 `R` 이 넘기는 시뮬 값이고, 코드 기본은 프로파일별 0.160(`legacy_160mm`) 또는 측정 260 mm 다 | `R:162-163`, `CPN:269-299`, `harvest_motion_params.py:68` |
| 가상 제어기 | cuRobo FK 의 ee + `tool_tcp_offset_m` 0.236 (툴 +Z) | `B:244,525-540` |
| Isaac 부착 기준 | `/World/robot_assembly/rh_p12_rn_base` | `I:287` |

**TcpTrail 제안**: `rh_p12_rn_base` 아래에 빈 자식 GameObject `tcp` 를 두고 ROS 로컬 (0, 0, +0.236) m 에 놓는다. URDF Importer 의 좌표 변환을 거치면 Unity 로컬 (0, 0.236, 0) 이 될 것으로 본다(단계 5에서 확인). 검산값: 영점 자세 TCP 는 ROS (0, 0, 1.359), 시작 자세 TCP 는 ROS (0.0226, 0.2151, 0.6426).

---

## 3. 플랜트 경계 계약

### 3-0. Isaac 안에서 도는 코드

| 코드 | 시작 | ROS 입출력 |
|---|---|---|
| `I` (`isaac_sim_bridge_node`) | Script Editor 에서 Run | **Isaac 의 ROS 입출력 전부**: 발행 2·구독 2 (`I:64-67,116-120`) |
| `isaac_sim_viewport_display.py` + `hud/*` | Script Editor 에서 Run | ROS 없음. `/tmp/harvest_hud_<role>.json` 읽기만 |
| `isaac_batch_orchestrator.py` | Script Editor 에서 한 번 Run | ROS 없음. `/tmp/harvest_batch/*.json` 파일 신호 |
| Kit 확장 `strawberry.sim.setup` | `scripts/run_isaacsim.sh` 의 `--enable` | ROS 없음. 화면 설정 |

씬·애셋 USD(텍스트 12개, 바이너리 9개 전부)에 OmniGraph·ROS 프림은 0개다. Isaac 의 ROS 입출력은 브릿지 스크립트 하나가 전부다.

### 3-1. Isaac 이 발행하는 토픽

| 토픽 | 타입 | 주기 | 내용 | stamp / frame_id |
|---|---|---|---|---|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | **물리 스텝마다**(PhysX 기본 60 Hz). 타임라인 Play 중에만 | `name` = §2-2 Isaac 순서 10개, `position` = **측정값** rad(목표값 아님), `velocity`·`effort` 빈 배열 | 벽시계 / 빈 문자열 |
| `/isaac_sim/strawberries` | `geometry_msgs/PoseArray` | 벽시계 1.0초 게이트(약 1 Hz). Play 중에만 | **익은 과실 루트 프림 원점(= 과실 중심)**, m, Z-up, orientation 항등. 부착 중·해제된 과실 제외. **빈 배열도 매번 발행** | 벽시계 / `"world"` |

근거: `I:602-678,680-704`, `strawberry_harvest/scenes/layers/physics_layer.usd:29-37`.

- 명령 적용이 실패해도 상태 발행은 계속한다(`I:602-608`). 소비자가 "팔이 안 움직였다"와 "브릿지가 죽었다"를 구분하게 하려는 설계다.
- 익음 구분은 프림 이름의 `unripe` 여부다(`I:640-649`). 안 익은 과실을 내면 플래너가 수확을 시도해 원본보다 관대해진다(코드 주석 `I:639-645`).
- 시연 배치 좌표(m, 익은 8개): ripe_01 (−0.25, 0.7828, 0.74) · 02 (−0.12, 0.7828, 0.76) · 03 (0.20, 0.7828, 0.74) · 04 (−0.22, 0.7828, 0.56) · 05 (−0.10, 0.7828, 0.44) · 06 (−0.25, 0.7828, 0.50) · 07 (−0.38, 0.7828, 0.72) · 08 (0.28, 0.7828, 0.70). 안 익은 4개(발행 안 함): unripe_02 (0.15, 0.7828, 0.90) · 04 (0.12, 0.7828, 0.56) · 05 (0.25, 0.7828, 0.48) · 06 (0.18, 0.7828, 0.38). 출처 `strawberry_harvest/scenes/layers/layout_layer.usd:228-321`.
- 과실 y 0.7828 = 보드면 0.810 − 27.2 mm. 비전 모킹이 첫 수신 때 평균 y 를 이 값과 비교해 20 mm 넘게 벗어나면 `BERRY_GEOMETRY` 경고를 낸다(`FV:119-132`).

### 3-2. Isaac 이 구독하는 토픽

**`/joint_command` (`sensor_msgs/JointState`)**

| 항목 | 동작 | 근거 |
|---|---|---|
| 수신 | 콜백은 `(name, position)` 을 덮어쓰기만 한다. 최신 1개만 남는다 | `I:83-90` |
| 적용 | 다음 물리 스텝에서 1회. 현재 측정값 배열을 복사한 뒤 **이름으로** DOF 를 찾아 덮어쓴다. 없는 DOF 는 현재값 유지, 모르는 이름은 무시(경고 1회) | `I:221-251` |
| 구동 | `apply_action(ArticulationAction(joint_positions=…))` = **드라이브 위치 목표**(rad). 텔레포트 아님 | `I:253-254` |
| 의미 | 한 메시지 = 한 시점의 설정점(궤적 아님). 보간은 가상 제어기가 한다 | `B:794-870` |
| 송신 빈도 | MoveJoint 5 ms 간격, MoveSplineJoint 2° 간격(≥3 ms), MoveLine IK 스텝마다(호출당 ≤24개), 그리퍼 1개. 유휴 중에는 안 보낸다 | `B:117,126,827,862-870,931-955` |

**`/sim/grasp_event` (`std_msgs/String`)**

| 문자열 | Isaac 이 하는 일 | 근거 |
|---|---|---|
| `"ATTACH x y z"` (`%.4f` m, base = world 프레임, 과실 중심) | 수확 안 된 익은 과실 중 가장 가까운 것을 찾는다. 60 mm 넘게 멀면 무시. 상대 변환을 유지한 채 그리퍼 밑동(`rh_p12_rn_base`)에 키네마틱으로 붙이고 줄기 조인트·콜라이더를 끈다 | `B:1090-1100`, `I:287-288,512-535` |
| `"RELEASE"` | 부착 해제, 그 과실을 `harvested` 에 넣는다. 트레이 상자 안이면 계란판으로, 밖이면 바닥으로 동적 낙하 | `B:1102-1110`, `I:537-572` |

### 3-3. Isaac 에 없는 것

| 항목 | 결론 |
|---|---|
| `/clock` | 발행 안 함 |
| TF (`/tf`, `/tf_static`) | 발행 안 함 |
| 서비스·액션 | 없음. 두산·그리퍼 서비스와 SafeGrasp 액션은 가상 제어기가 제공 |
| 카메라 이미지 토픽 | 없음. 손목 D455 는 뷰포트 표시 전용 |

### 3-4. ROS 가 아닌 경계

| 채널 | 방향 | 내용 | Unity 대체 |
|---|---|---|---|
| HUD 파일 버스 `/tmp/harvest_hud_{vision,controller,planner,scan}.json` | 노드 → Isaac 표시 스크립트 | 노드에 끼워 넣은 계측 코드가 0.1초마다 덮어쓴다. 위치는 `HARVEST_HUD_DIR_RUNTIME`(기본 `/tmp`) | 범위 밖. SimHud 가 연결·Hz·관절을 대신 보인다 |
| 배치 신호 `/tmp/harvest_batch/{request,isaac_state}.json` | `run_batch.sh` ↔ 오케스트레이터 | 재로드·Play 자동화 | 범위 밖 |
| Kit 로그 `[bridge] …` 줄 | Isaac → `run_batch.sh`·`run_metrics.py` | `PLACED in tray` / `DROPPED outside tray` / `DROP_REST` 를 문자열로 센다 | 범위 밖. 배치·낙하 판정은 Unity 가 재현하지 않는다 |

### 3-5. 가상 제어기 `sim_executor_bridge_node` (Unity 와 직접 맞닿는 상대)

**제공 서비스·액션**

| 이름 | 타입 | 읽는 요청 필드 | 응답 `success` | 근거 |
|---|---|---|---|---|
| `/dsr01/motion/move_joint` | `dsr_msgs2/srv/MoveJoint` | `pos[6]` deg 절대, `vel`, `acc`, `time` | **항상 True** | `B:829-878` |
| `/dsr01/motion/move_spline_joint` | `dsr_msgs2/srv/MoveSplineJoint` | `pos[]`(각 6개 deg), `time` | **항상 True** | `B:747-810` |
| `/dsr01/motion/move_line` | `dsr_msgs2/srv/MoveLine` | `pos[0:3]` mm 상대, `ref`(1 TOOL), `mode`(1만), `vel[0]` | IK 성공이면 True. IK 실패·`mode≠1`·예외면 False. **IK 솔버가 없으면 움직이지 않고 True** | `B:880-1039` |
| `/dsr01/motion/change_operation_speed` | `dsr_msgs2/srv/ChangeOperationSpeed` | 없음 | 항상 True(아무것도 안 함) | `B:1078-1080` |
| `/gripper_service/set_position` | `dsr_gripper_tcp_interfaces/srv/SetPosition` | `position` stroke 0–700 | 항상 True, 즉시 응답 | `B:1112-1133` |
| `/gripper_service/get_state` | `dsr_gripper_tcp_interfaces/srv/GetState` | 없음 | True, `present_position` = 판정 리드백 | `B:1135-1139` |
| `/gripper_service/safe_grasp` (액션) | `dsr_gripper_tcp_interfaces/action/SafeGrasp` | `target_position` | 판정, **판정 불가면 True**. **현행 런에서는 호출되지 않는다** | `B:1141-1162`, `planner_bootstrap.py:177` |

**도착 판정** (`B:681-743`)

| 항목 | 값 |
|---|---|
| 관절 상태 미수신 | 0.2초 뒤 `True` (판정 생략) |
| 오차 식 | `max_i |deg(측정_i) − 목표_i|`, 6축, **각도 감싸기 없음** |
| 도착 | 오차 ≤ 0.3° → `ARM_ARRIVAL` |
| 정체 | 0.15초 경과 뒤, 오차 ≤ 2.0° 이고 개선 < 0.01° 인 상태가 0.30초 → `ARM_ARRIVAL_STALLED` (도착으로 간주) |
| 타임아웃 | 8.0초 → `ARM_ARRIVAL_TIMEOUT` (오류 로그만, 시퀀스는 계속) |
| 폴링 | 0.02초 |

**파지 판정** (닫기 명령 순간 한 번, `B:545-616,1112-1133`)

1. `/dsr01/joint_states` 앞 6개 → cuRobo FK → TCP = ee + R·(0, 0, 0.236).
2. `/isaac_sim/strawberries` 각 과실 중심 + (0, 0, 0.035) = 줄기점. TCP 에 가장 가까운 것을 고른다.
3. 툴 +Z 방향 거리 `along` −6–27 mm, 수직 거리 `lateral` ≤ 20 mm → 접촉. 리드백 670, `ATTACH` 발행.
4. 아니면 빈손. 리드백 700.
5. 판정 불가(과실 좌표 없음·빈 배열·FK 불가) → 리드백 = 명령값 700 → 플래너 `GRASP_EMPTY` → 배치 게이트 차단.

플래너는 `GetState` 리드백 ≥695 를 `GRASP_EMPTY`, ≤685 를 `GRASP_CONTACT_DETECTED` 로 본다(`gripper_client.py:104-122`).

**실패 감지가 전부 플랜트의 측정값에 기대는 이유.** 보존 런 `run_logs/20260930_113910` 에서 플랜트 팔이 움직이지 않았다. 45·30·40 mm MoveLine 이 모두 '실제 0.0 mm' 였는데도 가상 제어기는 `MoveLine ok` 를 돌려줬다. 이 실패를 막은 것은 FK 기반 파지 판정 하나였다(along +64.6 mm, lateral 145.3 mm → EMPTY → `PLACE_GATE_BLOCKED`, `run_logs/20260930_113910/bridge.log:55-65`). Unity 가 명령을 그대로 상태로 돌려보내면 이런 실패를 잡을 수 없다(D-4).

### 3-6. Unity 가 Isaac 을 대신할 때 지킬 것

| 구분 | 항목 | 조건 |
|---|---|---|
| **필수** | `/dsr01/joint_states` 발행 | `JointState`, **앞 6칸 `joint_1..joint_6`**(권장: Isaac 순서 10개 그대로), **rad, 감싸지 않은 연속 각**(J1·J4·J6 는 ±180° 밖 값이 정상이다. 예: 복귀 종점 J4 −184.1°), **아티큘레이션 측정값**, 고정 주기로 끊김 없이(물리 스텝마다, 50–60 Hz). 유휴 중에도 계속 |
| **필수** | `/joint_command` 구독 | **이름으로** 매핑, **최신 1개만** 적용(쌓아 두고 재생하지 않음), 마지막 목표를 계속 유지, 없는 관절은 현재값 유지, 모르는 이름 무시, 드라이브 목표로 적용 |
| **필수** | 시작 자세 | 팔 overview `[87.98, −94.92, 129.89, 175.94, −31.34, 93.42]°`, 그리퍼 네 관절 53.05°(= 0.926 rad). 스캔 시작 게이트가 overview 와 1.0° 안을 요구한다(`SE:550,565-569`). 출처 `layout_layer.usd:19-97` |
| **필수** | 좌표계 | 로봇 베이스 = 월드 원점·회전 없음. ROS 쪽은 base_link = world, Z-up, 오른손, m. Unity(Y-up, 왼손)와 오갈 때 변환한다 |
| **필수**(M2) | `/isaac_sim/strawberries` 발행 | 익은 과실 중심, frame_id `"world"`, 약 1 Hz(벽시계). §3-1 고정 좌표에서 **붙은·수확된 과실을 뺀다**(Isaac 과 같다. 10-20 에는 정적 목록으로 정했다가 10-21 에 바꿨다, §9-4 D-6) |
| 처리(10-21 개정) | `/sim/grasp_event` | Isaac 과 같은 규칙으로 처리한다(§3-2 표): `ATTACH` 는 수확 안 된 익은 과실 중 60 mm 안의 최근접을 그리퍼 밑동에 붙이고, `RELEASE` 는 떼어 수확됨으로 표시한 뒤 놓인 자리에서 떨어뜨린다. 낙하는 운동학적(§9-4 D-6). 두 모드 모두(M1 은 bag 의 이벤트) |
| 필수 | ROS 설정 | 도메인 0, QoS 기본(depth 10, RELIABLE, VOLATILE) |
| 재현 안 함 | `/clock`·TF·서비스·액션 | Isaac 도 없다 |
| 재현 안 함 | 파지 판정 | 가상 제어기가 한다(§3-5) |
| 범위 밖 | 과실 낙하 물리·트레이 안착 판정·Kit 로그 줄·HUD 파일 버스·배치 오케스트레이터 | 화면·지표용 |
| 받지 않음 | `/dsr01/gripper/position_cmd` | 스캔이 내지만 시뮬에서는 아무도 안 받는다(실기 전용). Isaac 과 같게 무시 |
| 내면 안 됨 | `/dsr01/curobo/target_pose`, `/dsr01/curobo/obstacles` | 플래너가 구독만 열어 둔 입력이다. 내면 시퀀스 밖에서 계획하거나 장애물을 바꾼다 |

**발행 주기 하한이 있는 이유.** 도착 판정의 정체 조건은 0.30초 동안 오차가 안 줄면 도착으로 본다. 관절 상태가 약 3.3 Hz 보다 느리게 오거나 0.3초 넘게 끊기면, 아직 움직이는 팔을 최대 2° 오차로 '도착' 처리한다. 원본보다 관대해진다.

---

## 4. 시간

| 항목 | 값 | 근거 |
|---|---|---|
| `use_sim_time` | 아무 노드도 안 쓴다. `R` 의 `--ros-args` 에도 없다. 리포 안의 `use_sim_time` 은 빌드하지 않는 두산·ROBOTIS launch 뿐이다 | `R:132-196`, `docs/concepts.md:86` |
| `/clock` | 내는 쪽이 없다 | §3-3 |
| 노드의 시계 | 기본 ROS 시계 = 시스템 시간. 대기·타임아웃·페이싱은 거의 전부 `time.time()`·`time.sleep()` 벽시계 | `B:681-745,794-870`, `SE:715-725,1120-1128` |
| stamp | 발행 쪽이 `get_clock().now()` 로 채우지만, **받는 쪽은 stamp 를 읽지 않는다** | `I:620,633`, `B:656`, `FV:148` |
| Isaac 물리 | 60 Hz 고정 스텝(PhysicsScene 에 `timeStepsPerSecond` 미지정), 타임라인 60 | `physics_layer.usd:29-37`, `main_scene.usd:14` |
| 벽시계 기준 실제 Hz | 리포에 측정 기록 없음. 과실 토픽은 비전 모킹 경유 간접 측정으로 중앙값 1.009초 간격 | `log/m3/a4_record_20261006_0008/curobo_planner_node_*.jsonl` |

**Unity 시뮬레이터 결정**: Unity 는 `/clock` 을 내지 않고, 노드는 `use_sim_time:=false`(기본값) 그대로 둔다. 켜면 플래너의 2초 그리퍼 초기화 타이머와 HUD 타이머가 `/clock` 없이 멈춘다. 원본과 같은 시간 모델이 벽시계다.

---

## 5. Isaac 없이 띄우는 법

### 5-1. 노드 4개 (`R` 그대로)

| 순서 | 명령 | 작업 디렉터리 | 비고 |
|---|---|---|---|
| 0 | `source /opt/ros/humble/setup.bash && source install/setup.bash` | 딸기 리포 루트 | |
| 1 | `ros2 run strawberry_sim_core fake_vision_node` | 리포 루트 | 인자 없음 |
| 2 | `ros2 run strawberry_sim_core sim_executor_bridge_node` | 리포 루트 | 인자 없음. `tool_tcp_offset_m` 을 주면 안 된다 |
| 3 | 브릿지 로그에 `cuRobo IK Solver successfully initialized` 가 뜰 때까지 대기(최대 120초) | | CUDA 경합 회피 |
| 4 | `python3 curobo_planner_node.py --ros-args -p tool_model_profile:=legacy_160mm -p ee_to_tcp_offset_m:=0.236 -p enable_open_stem_descent:=true -p enable_straight_reverse_retreat:=true -p pick_target_z_bias_m:=0.035 -p allow_generated_tray_slot_release:=true -p enable_marker_place_sequence:=true -p use_taught_slot0_place_reference:=true -p taught_place_via_tray_view:=false -p execute_marker_place_release:=true -p hold_after_taught_slot0_place:=false -p hold_on_place_failure:=false -p taught_slot_sequence:=0,1,3,4,6,7,9,10` | `src/strawberry_motion/scripts` | src 원본을 직접 실행 |
| 5 | `python3 -m strawberry_motion.execution.scan_executor_node --ros-args -p execute_motion:=true -p target_cell:=all -p overview_prescan:=true -p scan_dwell_sec:=3.0 -p subdivide_min_candidates:=3 -p scan_movej_vel_deg_s:=120.0 -p scan_movej_acc_deg_s2:=180.0 -p overview_return_vel_deg_s:=120.0 -p overview_return_acc_deg_s2:=180.0` | 리포 루트 | install 사본 실행 |
| 6 | 플래너 로그에 `cuRobo Planner Ready!` 대기(최대 300초) | | |
| 7 | 트리거: `ros2 service call /strawberry/scan/start std_srvs/srv/Trigger` | | 플랜트가 overview 자세로 상태를 내는 중이어야 한다 |

- 노드 초기화 안에 다른 노드나 Isaac 을 기다리는 대기는 없다. 순서를 강제하는 것은 `R` 의 로그 문자열 대기 두 개뿐이다.
- GPU: 가상 제어기 IK 와 플래너 MotionGen 이 둘 다 `cuda:0` 을 쓴다. CPU 대체 경로는 없다.
- `R` 은 `check_params.py` 게이트도 돈다. Isaac 씬 USD 를 **텍스트로** 읽어 보드 y 등을 대조하므로 Isaac 없이도 통과한다. 그러나 Unity 씬이 맞는지는 검사하지 않는다.
- **주의: 이대로 띄우면 딸기 리포 안에 파일이 생긴다**(D-5). `R` 은 `run_logs/<STAMP>/` 와 `run_logs/latest` 를 만들고, 플래너는 JSONL 을 `src/e0509_gripper_description/logs/runtime/<날짜>/` 에 쓰고, `__pycache__` 가 생긴다. 모두 딸기 리포의 `.gitignore` 대상이지만 "파일 생성 금지" 규칙에는 걸린다. 피하는 방법은 §9 D-5.

### 5-2. 노드별 Isaac 입력

| 노드 | 직접 받는 Isaac 입력 | 다른 노드를 거쳐 받는 입력 | 그 입력이 없으면 |
|---|---|---|---|
| 비전 모킹 `FV` | `/isaac_sim/strawberries` | 스캔의 `set_cell_state`(시야 경계) | 아무것도 발행하지 않는다(콜백 구동뿐) |
| 가상 제어기 `B` | `/dsr01/joint_states`, `/isaac_sim/strawberries` | — | 이동 서비스가 0.2초 뒤 성공. 파지 판정 불가 → `GRASP_EMPTY` |
| 플래너 `CPN` | `/dsr01/joint_states` (이름 매핑) | `scene_positions`(비전 경유, 이웃 장애물), 파지 판정(가상 제어기 경유) | `pick_pose` 를 받아도 `No joint state yet` 로 버리고 `pick_complete` 를 안 낸다 |
| 스캔 `SE` | `/dsr01/joint_states` (이름 매핑, 시작 게이트·도착 확인) | `pick_pose`·`scene_positions`(비전 경유), `pick_complete`(플래너) | 트리거가 `START_REJECTED no current joint state received` 로 거부된다 |

비전 모킹 가공: 0.5초 스로틀(입력이 1 Hz 라 실제 출력도 약 1 Hz) → 시야 필터(`VIEWING` 이면 그 칸, 그 밖 상태는 부모 분면, `root` 는 전부; x·z 만 본다. 보드 x [−0.495, 0.595], z [0.265, 1.055], 분면 중심선 x 0.050·z 0.660) → 좌표를 **그대로** `pick_pose`(frame_id 를 `base_link` 로 바꿔 씀)와 `scene_positions` 로 낸다(`FV:103-154`, `quadrant_filter.py:22-26,52-101`).

### 5-3. 실행 흐름 (트리거부터 한 과실까지)

| # | 주체 → 대상 | 이름 | 내용 |
|---|---|---|---|
| 1 | 운영자 → 스캔 | `/strawberry/scan/start` | 게이트: 관절 상태 수신 → overview ±1.0°(J1·J4·J6 는 360° 등가 허용) |
| 2 | 스캔 → 비전 | `/strawberry/exploration/set_cell_state` `root=SCANNING` | 시야 해제 |
| 3 | 비전 → 스캔 | `/strawberry/detection/scene_positions` | 3초 동안 분면별 개수 집계 → 0개 분면 가지치기 |
| 4 | 스캔 → 가상 제어기 | `/dsr01/motion/move_joint` | 분면 스캔 자세로(vel 120, acc 180) |
| 5 | 가상 제어기 → 플랜트 | `/joint_command` | 사다리꼴 보간 스트림 → 도착 대기 |
| 6 | 플랜트 → 스캔 | `/dsr01/joint_states` | 스캔이 ±3.0° 도착을 최대 90초 확인 |
| 7 | 스캔 → 비전 | `set_cell_state` `<cell>=VIEWING` | 3초 동안 `pick_pose` 수집 → 30 mm 중복 제거 → 후보 ≥3 이면 세부 칸으로 분할 |
| 8 | 스캔 → 플래너 | `/dsr01/curobo/pick_pose` | 타겟 하나 |
| 9 | 플래너 → 가상 제어기 | `change_operation_speed`, `set_position`(600), `move_spline_joint`(프리어프로치), `move_line`(진입·하강) | |
| 10 | 플래너 → 가상 제어기 | `set_position`(700) → `get_state` | 닫기·판정. 접촉이면 `/sim/grasp_event` `ATTACH` |
| 11 | 플래너 → 가상 제어기 | `move_line`(당김·후퇴), 접촉이면 배치(`move_spline_joint`·`move_line`·`set_position`(600) → `RELEASE`) | |
| 12 | 플래너 → 스캔 | `/dsr01/curobo/pick_complete` | 성공·실패 구분 없음. 스캔은 최대 60초(+재발행 60초) 기다린다 |
| 13 | 스캔 → 가상 제어기 | `move_joint`(overview) | 끝나면 `SCAN_COMPLETE` → `READY_FOR_NEXT_START` |

**픽 하나의 시간 예산이 빠듯하다.** 보존 런 전체에서 `PICK_TRIGGER` 부터 완료까지 중앙값 31.9초, p95 36.8초, 최대 54.1초였다. 스캔의 대기 상한은 60초다. 넘으면 같은 타겟이 다시 발행되고 플래너 대기열 때문에 `pick_complete` 가 한 칸씩 밀린다. Unity 의 추종이 Isaac 보다 느리면 이 문제가 바로 드러난다.

### 5-4. Isaac 없이 띄우면 어디서 멈추나

| 경우 | 결과 |
|---|---|
| A. 플랜트가 아무것도 안 낸다 | 기동·기동 로그 대조는 전부 통과한다. 트리거가 `START_REJECTED no current joint state received` 로 거부된다. 그 전에 플래너가 기동 2초 뒤 `set_position(600)` 을 부르면 가상 제어기가 팔 `[0,0,0,0,0,0]` 이 실린 `/joint_command` 를 낸다(`CPN:529-531,559-565`, `B:509-513`) |
| B. 관절 상태만 있다(overview) | 트리거 수락 → 과실 0개 → 네 분면 가지치기 → `SCAN_COMPLETE`. 픽 0회 |
| C. 둘 다 있지만 관절 상태가 명령을 따라오지 않는다 | 첫 MoveJoint 에서 가상 제어기가 8초 기다린 뒤 성공 응답 → 스캔이 90초 기다린 뒤 `EXEC_TIMEOUT` → overview 복귀 시도 → 시퀀스 중단 |

**기동 순서 계약**: 플랜트(Unity Play, 관절 상태 발행)를 먼저 올리고 노드를 띄운다. A 의 영점 명령이 플랜트에 적용되면 팔이 영점 자세로 가서 시작 게이트가 거부한다.

---

## 6. M2 의존성 맵 — Isaac 을 끄면 끊기는 것

| Isaac 이 하던 일 | 끊으면 | 대안 | 권장 |
|---|---|---|---|
| `/dsr01/joint_states` 발행 | 스캔 시작 거부. 가상 제어기 도착 판정 생략, 파지 판정 FK 불가 | Unity 가 대신 발행 | **Unity 발행(필수)**. 측정값, 연속 각, 앞 6칸 순서 |
| `/joint_command` 적용 | 팔이 안 움직인다 | Unity 가 구독해 드라이브 목표로 적용 | **Unity 구독(필수)**. D-4 |
| `/isaac_sim/strawberries` 발행 | 과실 0개 → 픽 0회(5-4 B). 파지 판정 불가 → 빈손 | ① Unity 가 정적 배치(§3-1 좌표) + 부착·해제 과실 제외로 발행 ② H4 bag 에서 이 토픽만 재생 ③ 범위 밖(스캔 이동까지만 시연) | **① (10-21 부터 부착·해제 과실 제외까지, §9-4 D-6)**. ② 는 Isaac 런의 시각에 묶여 있어 부착 뒤 목록에서 빠지는 시점이 Unity 시뮬레이터 런과 어긋난다 |
| `/sim/grasp_event` 처리 | ROS 쪽 흐름은 안 끊긴다(발행만 하는 토픽). 다만 부착 과실이 과실 목록에 남는다 | Unity 가 ATTACH 때 최근접 과실을 그리퍼 자식으로 붙이고 목록에서 뺀다. RELEASE 때 떼어 둔다 | **처리(10-21 개정, `GraspFollower`)**. 10-20 에는 처리하지 않기로 했다(스캔 실행기가 시도 좌표를 25 mm 블랙리스트로 걸러 흐름은 이어졌다) |
| 파지 판정 | Isaac 은 원래 안 한다 | — | 가상 제어기가 그대로 한다. Unity 재현 불필요 |
| 과실 낙하·트레이 안착, Kit 로그 지표 | `run_metrics.py` 집계 불가 | — | 화면의 낙하는 운동학적으로 보인다(D-6). Kit 로그 지표·`run_metrics.py` 집계는 **범위 밖**. 그래서 "수확 성공"이라는 말은 쓰지 않는다 |
| HUD 표시(파일 버스 읽기) | 화면만 없어진다 | SimHud | **10-21 추가**: `scripts/hud_relay.py` 가 딸기 HUD 모듈(`bus_merge`·`tree_model`·`result_bar`·`status_bus`)을 import 만 해 같은 계산을 하고, Unity `HarvestHud`·`BoardDisplay` 가 그린다(패널·보드 하이라이트·방위 글자·손목 카메라 창·미니맵) |
| 씬 리셋(재로드) | — | Unity Play 재시작 = overview 자세·과실 배치 복원 | Unity |
| 배치 오케스트레이터 | `run_batch.sh` 자동 런 불가 | — | 범위 밖 |
| `check_params.py` | 영향 없음(USD 텍스트만 읽음) | — | 그대로. 단 Unity 씬 정합은 따로 확인 |

**경로 판단(처음 계획 단계 7의 가·나).** Isaac 에서 오던 팔 외 입력은 `/isaac_sim/strawberries` 하나다. 파지 판정은 ROS 쪽에 있다. 그래서 **(가) 전체 스택이 가능하다**. Unity 가 정적 과실 좌표를 내면 bag 없이도 플래너 → 가상 제어기 → Unity → 상태 회신 루프가 닫힌다. (나) 시퀀스 스크립트는 rclpy 로 쓰면 되고, 어느 경로든 Unity 쪽에 커스텀 메시지가 필요 없다(§8). 최종 선택은 단계 7에서 한다.

**M2 완료 판정 방법.** 서비스 응답은 도착과 무관하게 성공이라 근거가 안 된다. 다음 세 가지로 판정한다.

- 가상 제어기 로그의 `ARM_ARRIVAL` / `ARM_ARRIVAL_STALLED` / `ARM_ARRIVAL_TIMEOUT` 줄 수(타임아웃 0 이 목표).
- 스캔 로그의 스캔 자세 도착 확인 통과와 `SCAN_COMPLETE`.
- 스캔 로그의 `PICK_TRIGGER` 가 1회 이상이고 시작한 픽이 전부 `PICK_ATTEMPT_DONE` 으로 끝남. overview 스캔에서 익은 과실이 0개면 스캔 실행기는 스캔 자세에 가지 않고 3 s 만에 `SCAN_COMPLETE` 를 내므로, 이 조건 없이는 픽 없는 런이 완주로 잡힌다(`logs/m2/20261022_003302_m2_editor`, `scripts/m2_summary.py` 의 `picks_attempted`).

---

## 7. ROS 환경

| 항목 | 값 | 근거 |
|---|---|---|
| 배포판 | Humble (`/opt/ros/humble`, 시스템 Python 3.10) | |
| 딸기 워크스페이스 | `/home/oceanshape/strawberry_grasp_environment/install/setup.bash` | `R:83` |
| install 패키지 | `dsr_gripper_tcp_interfaces`, `dsr_msgs2`, `e0509_gripper_description`, `strawberry_motion`, `strawberry_sim_core` (5개) | `ls install/` |
| 설치 방식 | `strawberry_motion`·`strawberry_sim_core` 는 **복사 설치**. src 를 고치면 colcon build 를 다시 해야 반영된다(문서의 `--symlink-install` 서술과 다름) | `find install -name '*.egg-link'` |
| `ROS_DOMAIN_ID` | **어디서도 설정 안 함 → 0**. 딸기 리포, `~/.bashrc`, `~/.profile`, `/etc` 모두 0건 | |
| `RMW_IMPLEMENTATION` | 노드 쪽 미설정(설치된 RMW 가 `rmw_fastrtps_cpp` 하나뿐). Isaac 쪽은 `rmw_fastrtps_cpp` 명시 | `scripts/run_isaacsim.sh:60` |
| `ROS_LOCALHOST_ONLY` | 0 (끔) | |
| Fast DDS | 시스템 2.6.11, Isaac 번들 2.6.10. 같은 계열이라 토픽이 오간다 | |
| Fast DDS XML 프로파일 | 없음 | |
| QoS | 경계 토픽 전부 depth 10, RELIABLE, VOLATILE. 예외는 비전 모킹의 `pick_pose` 발행(depth 20) | `I:65-67,120`, `B:287-297`, `FV:40-41` |

### 7-5. 녹화물과 H4 녹화 명령

- `~/bags` 는 비어 있다. 딸기 리포와 홈에 `.db3`·`.mcap`·`metadata.yaml` 이 없다. `run_logs/` 는 노드 stdout 텍스트뿐이다.
- rosbag2 0.15.16(sqlite3) 이 설치돼 있다. mcap 플러그인은 없다. Humble rosbag2 는 서비스 호출을 녹화하지 않는다.
- bag 이 없어도 관절 목표의 텍스트 기록은 있다. 가상 제어기 로그에 `MoveJoint called to [deg×6]`, `MoveSpline ok … end=[…]`, `MoveLine ok` 줄이 남아 있다(0.1° 자리까지). (나) 경로의 대체 입력으로 쓸 수 있다.

**H4 녹화 명령(처음 계획 단계 6 수정판).** 경계 토픽이 전부 표준 타입이라 `/opt/ros/humble` 만 소싱해도 된다.

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/run1 \
  /dsr01/joint_states /joint_command /isaac_sim/strawberries /sim/grasp_event \
  /strawberry/detection/pick_pose /strawberry/detection/scene_positions \
  /strawberry/exploration/set_cell_state /strawberry/scan/status \
  /dsr01/curobo/pick_pose /dsr01/curobo/pick_complete /dsr01/gripper/position_cmd
```

녹화하는 동안 다른 터미널에서 `/dsr01/joint_states` 의 벽시계 Hz 를 한 번 잰다. 리포에 측정 기록이 없는 값이다.

```bash
ros2 topic hz /dsr01/joint_states
```

---

## 8. 커스텀 메시지

### 8-1. 경계별 타입

| 경계 | 이름 | 타입 | 표준/커스텀 |
|---|---|---|---|
| Isaac ↔ 노드 | `/dsr01/joint_states`, `/joint_command` | `sensor_msgs/msg/JointState` | 표준 |
| Isaac ↔ 노드 | `/isaac_sim/strawberries` | `geometry_msgs/msg/PoseArray` | 표준 |
| Isaac ↔ 노드 | `/sim/grasp_event` | `std_msgs/msg/String` | 표준 |
| 플래너·스캔 ↔ 가상 제어기 | `/dsr01/motion/{move_joint,move_spline_joint,move_line,change_operation_speed}` | `dsr_msgs2/srv/{MoveJoint,MoveSplineJoint,MoveLine,ChangeOperationSpeed}` | 커스텀 |
| 플래너 ↔ 가상 제어기 | `/gripper_service/{set_position,get_state}`, `/gripper_service/safe_grasp` | `dsr_gripper_tcp_interfaces/srv/{SetPosition,GetState}`, `…/action/SafeGrasp` | 커스텀 |
| 노드 내부 | `pick_pose`, `scene_positions`, `pick_complete`, `scan/status`, `set_cell_state`, `gripper/position_cmd`, `/strawberry/scan/start` | `PoseStamped`, `Float64MultiArray`, `Empty`, `String`, `Int32`, `std_srvs/srv/Trigger` | 표준 |

**결론: Isaac 자리(Unity)에는 표준 타입만 오간다.** 커스텀 타입은 모두 플래너·스캔과 가상 제어기 사이(둘 다 시스템 rclpy 노드)에서만 쓰인다. 그래서 처음 계획 단계 7의 "커스텀 메시지 C# 생성"은 Unity 가 두산 서비스를 직접 부르거나 제공할 때만 필요하다. (가)·(나) 어느 경로든 필요 없다. `/strawberry/scan/start` 를 Unity 에서 부르고 싶을 때만 `std_srvs/Trigger` 생성이 필요할 수 있다.

### 8-2. 정의 폴더

| 패키지 | 폴더 | 파일 수 |
|---|---|---|
| `dsr_msgs2` 1.1.0 | `src/doosan_robot2/dsr_msgs2/msg/` | 18 `.msg` |
| | `src/doosan_robot2/dsr_msgs2/srv/` (하위 폴더 12개: motion 27, force 23, aux_control 18, realtime 16, system 15, io 10, plc 9, drl 5, tool 5, gripper 4, modbus 4, tcp 4) | 140 `.srv` |
| | `src/doosan_robot2/dsr_msgs2/action/` | 3 `.action` |
| `dsr_gripper_tcp_interfaces` | `src/dsr_gripper_tcp_interfaces/{msg,srv,action}/` | 1 + 2 + 1 |

- 경계에 쓰이는 정의: `srv/motion/MoveJoint.srv:6-15`, `srv/motion/MoveLine.srv:5-16`, `srv/motion/MoveSplineJoint.srv:6-14`, `srv/motion/ChangeOperationSpeed.srv:5-7`, `dsr_gripper_tcp_interfaces/srv/SetPosition.srv:1-5`, `srv/GetState.srv:1-4`, `msg/GripperState.msg:1-6`, `action/SafeGrasp.action:1-11`.
- 설치본은 `.srv` 를 하위 폴더 이름 아래(`install/dsr_msgs2/share/dsr_msgs2/motion/MoveJoint.srv`)에 둔다. 그래서 `ros2 interface show dsr_msgs2/srv/MoveJoint` 가 `InvalidResourceName: //` 로 실패한다. 파이썬 import 는 정상이다.
- ROS-TCP-Endpoint 가 import 할 파이썬 생성물: `install/dsr_msgs2/local/lib/python3.10/dist-packages/dsr_msgs2/`, `install/dsr_gripper_tcp_interfaces/local/lib/python3.10/dist-packages/dsr_gripper_tcp_interfaces/`. 딸기 `install/setup.bash` 를 소싱하면 `PYTHONPATH` 에 들어간다. 엔드포인트도 시스템 Python 3.10 으로 띄운다.
- ROS-TCP-Connector v0.7.0 의 내장 메시지(std·geometry·sensor·rosgraph·builtin_interfaces 등)로 경계 타입이 전부 덮인다. 내장 목록은 일반 지식이라 단계 4에서 `Library/PackageCache/` 소스로 확인한다.

---

## 9. 처음 계획과 어긋나는 점·결정 항목

### 9-1. 그대로 고치면 되는 것 (사실 정정)

| 처음 계획 | 실제 | 반영할 곳 |
|---|---|---|
| JointStateMirror 기본 토픽 `/joint_states` | `/dsr01/joint_states` | 단계 5 기본값 |
| H4 녹화 `ros2 bag record … /joint_states …` | §7-5 명령 | H4 안내 |
| 단계 6 "Hz≈50" | Isaac 은 물리 스텝마다(60 Hz 기본). 벽시계 Hz 는 미측정 | M1 확인 기준 |
| 단계 7 "커스텀 메시지 C# 생성" | 경계에 커스텀 타입 없음 | 단계 7 생략 가능 |
| 단계 7 "ClockPublisher 또는 use_sim_time:=false" | 아무도 sim time 을 안 쓴다 → `/clock` 안 냄 | 단계 7 |
| 단계 7 "파지 판정: bag 판정 토픽 재생 / 판정 없는 구간까지만" | 판정은 가상 제어기가 한다. Unity 는 측정값과 과실 좌표만 내면 된다 | 단계 7 |
| 단계 7 "가상 제어기의 도달 판정이 Unity 상태로 통과" | 응답이 아니라 로그 줄로만 확인된다 | M2 완료 기준(§6) |
| 단계 6 fake 퍼블리셔 "±30° 사인파, 한계 안쪽" | J2 시작값 −94.92° 는 한계까지 0.08° 다. 사인파 중심을 시작 자세로 두면 J2·J3 가 한계를 넘는다 | 단계 6: 중심을 한계 안쪽으로 잡거나 진폭을 관절별로 줄인다 |

### 9-2. 정할 것

**D-1. Unity URDF 의 원천 (단계 3).** 처음 계획은 "CONTRACT 1번 명령으로 생성"이라 했다. 그러나 xacro 결과는 J2·J3·J5 한계가 Isaac 보다 넓고 커스텀 파츠가 6° 벌어져 있다(§1-3). **권장: 리포 `robot.urdf` 에서 사본을 만들고 §1-4 의 세 가지만 고친다.** 대안은 xacro 결과에 같은 손편집을 다시 넣는 것인데, 결과가 같고 단계만 많다.

**D-2. 그리퍼 `rh_r2`·`rh_l2` 상한 (단계 3).** 실기·URDF 는 1.0 rad, Isaac 조립본·cuRobo 는 1.1 rad 다. 가상 제어기가 1.08 rad 를 명령한다. **권장: Isaac 과 같은 1.1.** 1.0 이면 두 관절만 잘려 손가락이 엇갈린다. 이 차이는 딸기 프로젝트가 '의도한 시뮬 편차'로 기록해 둔 것이다(`docs/parameters.md:157-164`).

**D-3. 관절 한계 판정 여유 (단계 5).** 정상 시퀀스가 J3 135.0° 까지 쓰고 시작 자세 J2 가 한계에서 0.08° 안쪽이다. **권장: 0.01° 정도의 작은 여유**를 두고, 여유 안쪽 접근은 '경계 접근' 정보로 따로 센다.

**D-4. M2 Command 모드의 구동 방식 (단계 5·7).** 처음 계획의 공용 적용 함수는 `xDrive.target` 과 `jointPosition` 을 **둘 다** 쓴다. 이것은 텔레포트라서 Command 모드에 쓰면 측정값이 언제나 명령과 같아진다. 그러면 도착 판정이 항상 즉시 통과하고, 파지 판정이 '실행'이 아니라 '계획'을 재게 된다. 플랜트가 안 움직인 실패(§3-5 의 09-30 런)를 잡지 못하니 원본보다 관대하다. **권장**:

- Mirror 모드(M1, 측정값 재생): 지금처럼 둘 다 써도 된다. 받는 값이 이미 측정값이다.
- Command 모드(M2): `xDrive.target` 만 쓰고 `jointPosition` 은 시작 자세 초기화 때만 쓴다. 회신은 ArticulationBody 가 실제로 움직인 `jointPosition`(측정값)으로 한다. Isaac 과 같은 방식이다(드라이브 목표 → 물리 스텝 → 측정값 발행).

문서에서 쓰는 "운동학 구동(동역학 검증은 범위 밖)" 은 드라이브로 따라가되 접촉·하중을 다루지 않는다는 뜻이다.

**D-5. 딸기 노드 실행 방식 (H4 전, 단계 7).** `R` 으로 띄우면 딸기 리포 안에 `run_logs/`, 플래너 JSONL, `__pycache__` 가 생긴다(전부 딸기 리포의 `.gitignore` 대상). 이 작업의 규칙은 딸기 리포에 "파일 하나도 생성하지 않는다"다. **권장**: Unity 시뮬레이터 리포에 `R` 과 같은 인자로 노드를 띄우는 스크립트를 만든다. 로그는 Unity 시뮬레이터 리포 `logs/` 로 보내고 아래 환경 변수를 준다.

| 환경 변수 | 값 | 막는 것 |
|---|---|---|
| `STRAWBERRY_RUNTIME_LOG_ROOT` | `~/e0509_unity_sim/logs/runtime` | 플래너 JSONL |
| `PYTHONDONTWRITEBYTECODE` | `1` | `__pycache__` |
| `HARVEST_HUD_DIR_RUNTIME` | 기본 `/tmp` 유지 | (딸기 리포 밖이라 그대로) |

`HARVEST_HUD_DIR` 는 바꾸지 않는다. 계측 코드 import 경로라서 바꾸면 계측이 조용히 꺼진다. H4 녹화도 같은 문제가 있다. Isaac 런을 평소처럼 `R` 으로 하면 `run_logs/` 가 생긴다. H4 를 평소 운용으로 보고 허용할지, Unity 시뮬레이터 리포의 스크립트로 띄울지 정해야 한다.

**D-6. M2 의 과실 좌표 공급원 (단계 7).** (10-21 에 이 권장안대로 개정됐다, §9-4) **권장: Unity 가 §3-1 의 정적 배치 좌표를 1 Hz 로 내고, ATTACH·RELEASE 된 과실을 뺀다.** bag 재생은 Isaac 런의 시각에 묶여 있어 부착 뒤 목록에서 빠지는 시점이 Unity 시뮬레이터 런과 어긋난다. Unity 가 발행해도 판정은 가상 제어기가 그대로 하므로 "항상 성공" 스텁이 아니다.

**D-7. 스캔 MoveJoint 속도 (단계 7).** `R` 은 스캔·overview 복귀를 120 °/s·180 °/s² 로 덮어쓴다. 코드 기본값(실기 원본)은 60/90 이다. 이 값은 두산 MoveIt 가속도 한계를 넘고, 가상 제어기는 자르지 않고 `MOVEJ_OVER_DOOSAN_MOVEIT` 로 기록만 한다. 딸기 프로젝트가 '시뮬 전용 의도적 차이'로 결정해 둔 값이다(`docs/parameters.md` §8). **권장: Isaac 기준 런과 비교할 수 있게 120/180 을 유지하고 차이로 기록한다.**

### 9-4. 확정된 결정 (2026-10-20)

정한 결정 1–6 과 D-1–D-7 을 합친 것이다. 처음 계획과 이 문서의 다른 절이 이 표와 다르면 이 표를 따른다.

| 항목 | 확정 내용 | 반영 |
|---|---|---|
| D-1 (결정 1) | Unity URDF 원천은 딸기 리포 `robot.urdf` 사본이다. §1-4 의 세 가지만 고친다. 관절 한계와 그리퍼 상한은 Isaac 과 같아야 한다. 원천 경로·커밋 해시는 §1-4 에 적는다. JointLimitMonitor 도 이 한계를 쓴다 | 단계 3·5 |
| D-2 | `rh_r2`·`rh_l2` 상한 1.1 rad. Isaac 조립본과 같은 값이다 | 단계 3 |
| D-3 | 위반은 **여유 없이** `\|q\| > 한계` 로 센다. 한계 근처 접근(안쪽 0.5° 이내)은 위반과 따로 기록한다. 0.01° 여유 권장안은 원본보다 관대해서 버렸다 | 단계 5 |
| D-4 (결정 2) | Command 모드는 드라이브 목표만 주고 실제로 움직인 값을 회신한다. 중력 off, 충돌 off, 강성·감쇠는 수렴이 보장되는 값. 도착 판정은 가상 제어기 로그로 하고 그 로그를 `logs/m2/` 에 복사해 근거로 쓴다. 2시간 안에 0.3° 수렴이 안 되면 운동학 대입으로 폴백하고 README 한계에 "도착 판정 미검증"을 적는다(이 폴백 규칙은 이 프로젝트에서 유지) | 단계 4·5·7 |
| 결정 3 | `/dsr01/joint_states` 는 Isaac 발행 순서(§2-2)를 고정 배열로 낸다. 이름 매핑은 받는 쪽(미러·명령)에만 쓴다 | 단계 7 |
| D-5 (결정 4) | 딸기 노드는 이 리포의 실행 스크립트로 띄운다. 런 로그·캐시는 환경 변수와 실행 위치로 딸기 리포 밖에 둔다(§9-2 D-5 표). "딸기 리포 변경 없음"의 정의는 넓히지 않는다. 런 로그·캐시도 딸기 리포 안에 만들지 않는다 | H4·단계 6·7 |
| 결정 5 | 미러 기본 토픽 `/dsr01/joint_states`, H4 녹화는 §7-5 명령. 처음 계획을 §9-1 사실에 맞게 고쳤다: 단계 7의 판정 처리 선택지 삭제, 커스텀 메시지 생성 삭제, M2 완료 기준은 가상 제어기 로그의 도착 기록 + 시퀀스 완주 | 처음 계획 |
| D-6 (과실 모델) | **10-21 개정 — 잡은 딸기가 그리퍼를 반드시 따라가도록.** Unity 가 `/sim/grasp_event` 를 Isaac 브릿지와 같은 규칙으로 처리한다(`GraspFollower`, `I:287-291,512-572`): ATTACH = 수확 안 된 익은 과실 중 최근접, 60 mm 밖·이미 부착이면 무시, 붙는 순간의 그리퍼 기준 상대 자세로 `rh_p12_rn_base` 를 따라간다. RELEASE = 해제·수확됨 표시, 트레이 상자(수평 ±30 mm, 윗면 위 0.30 m 이하) 안이면 배치·밖이면 낙하로 센다. 붙은·수확된 과실은 `/isaac_sim/strawberries` 에서 뺀다(Isaac 과 같다). Isaac 과 남는 차이 하나: **낙하가 운동학적이다** — 강체로 바꾸지 않고 과실 정점마다 아래 계란판·책상·상자 면까지의 틈 중 최소만큼 중력 가속으로 내린다(굴러가지 않는다). 콜라이더를 더하지 않아 로봇 동작에 영향이 없다. 10-20 첫 결정(정적 목록, 이벤트 무시)에서는 수확된 과실이 플래너의 이웃 장애물로 남았다 — 개정으로 그 차이는 없어졌고, M2 결과가 바뀔 수 있어 다시 돌렸다(README 측정치) | 단계 7·8 |
| D-7 | 스캔·overview 복귀 MoveJoint 는 실기 원본 60 °/s·90 °/s² 로 돈다. 네 값(`scan_movej_vel_deg_s`·`scan_movej_acc_deg_s2`·`overview_return_vel_deg_s`·`overview_return_acc_deg_s2`)은 스캔 실행기의 ROS 파라미터이고 코드 기본값이 60/90 이다(`SE:136-139,218-225`, install 사본도 같음). 그래서 딸기 리포를 고치지 않고 실행 스크립트 인자로 넘긴다. 런치 인자로 바꿀 수 있는 값이라 README 한계 기록 대상이 아니다. 딸기 `run_nodes.sh` 로 돈 과거 Isaac 런은 120/180 이었다 | 단계 6·7 |
| 결정 6 | §9 의 나머지는 권장안대로 간다. 단, 권장안이 (a) 시뮬을 원본보다 관대하게 만들거나 (b) 딸기 리포 소스를 고치거나 (c) 하루 이상 걸리면 멈추고 묻는다 | — |

### 9-3. 단계 4·5에서 확인할 열린 질문

| 질문 | 확인 방법 |
|---|---|
| URDF Importer v0.5.2 가 ±360° revolute 한계를 ArticulationBody 에 그대로 넣는가 | 임포트 뒤 각 관절의 `xDrive.lowerLimit/upperLimit` |
| ArticulationBody 의 `jointPosition` 이 J1·J4·J6 를 ±180° 로 감싸는가 | 175.94° 근처에서 J4 를 넘겨 보고 읽기값 확인. 감싸면 발행 전에 이전 값 기준으로 펼쳐야 한다 |
| DAE 노드 행렬(mm 이동, `E0509_3_2` 회전)과 scale 0.001 조합이 Isaac 처럼 놓이는가 | 영점 자세에서 플랜지 z 1.123 m 확인 |
| 이름이 `world` 인 빈 루트 링크(inertial 없음)를 임포터가 어떻게 다루는가 | 문제가 되면 사본에서 `world` 를 빼고 `base_link` 를 루트로 |
| ROS-TCP-Connector v0.7.0 내장 메시지 목록 | `Library/PackageCache/com.unity.robotics.ros-tcp-connector*/Runtime/Messages/` |
| `/dsr01/joint_states` 의 벽시계 Hz (Isaac) | H4 때 `ros2 topic hz` |
| 최상위 `<material>` 이름 중복(`black`·`white`·`orange` 가 두 번씩)을 임포터가 받아들이는가. `black`·`white` 는 두 정의의 색이 같고 `orange` 는 어떤 링크도 쓰지 않는다 | 단계 4 임포트 로그. 오류가 나면 `make_unity_urdf.py` 에서 중복을 하나로 합친다(화면 영향 없음) |

**단계 4 결과 (2026-10-20, `logs/step4/04–06_build_scene*.extract.log`).**

- ±360° revolute 한계: 임포터가 `xDrive.lowerLimit/upperLimit` 에 ±360.0009° 로 그대로 넣는다(6.2832 rad × Rad2Deg). PhysX 가 거부하거나 깎지 않았다. 런타임에서 한계 근처 동작은 단계 5에서 본다.
- `jointPosition` ±180° 감싸기: **감싸지 않는다.** M1 스모크 테스트에서 J4 가 145.94–205.94° 를 오가는 동안 측정값이 그대로 이어졌다(`logs/m1/20261020_202903_fake/smoke_report.json`, wrap=False). JointLimitMonitor 의 펼침 코드는 안전장치로 남긴다.
- PhysX 관절 한계는 부드럽다: Mirror 로 137° 를 대입하면 135.57° 에서 멈춘다(한계 밖 0.57°). 여유 0 판정(D-3)이라 위반으로 센다. Command 모드에서 한계 근처 목표(J3 135.0°, J2 −94.92°)에 오버슈트가 나면 Isaac 에 없는 위반이 생기므로 단계 7 수렴 시험에 포함한다.
- DAE 노드 행렬·스케일: **문제가 실제로 났다.** 두산 .dae 는 정점이 로봇 전역 영점 프레임(mm)이고 노드 `<matrix>` 가 링크 프레임으로 되돌리는 구조인데(E0509_1_x −204.5, 2_x +119.5, 4_x −950.5, 6_1 −1123 mm, 3_2 는 X축 90° 회전), Unity Collada 임포트 + URDF-Importer `ColladaAssetPostProcessor` 는 이 노드 이동을 정점과 다른 축으로 합성해 link_4·link_6 시각이 ~1.1 m 어긋났다. 해결: `scripts/bake_dae_node_transforms.py` 가 사본의 노드 행렬을 정점·법선에 베이크하고 단위행렬로 둔다(28개 중 20개, `make_unity_urdf.py` 단계 5로 통합). 결과 정점 집합은 Isaac(assimp)이 보는 것과 같다. 스케일 0.001 은 그대로 맞았다(영점 FK 플랜지 y 1.123 m, 전체 bounds 높이 1.357 m).
- 시각 메쉬 축: 임포터의 `chosenAxis` 는 Visual/Collision 래퍼에 Euler(−90,0,90) 을 곱하는 것뿐이고, .dae(Z_UP 후처리)·.stl(Ros2Unity) 변환이 이미 적용돼 있어 **yAxis(추가 회전 없음)가 맞다**. 단위행렬 메쉬(base)의 치수가 yAxis 에서만 파일과 일치한다. 처음 계획 단계 4의 'Z-axis' 는 이 파일들에는 틀린 가정이었다. `SimSetup.ChooseVisualAxis` 가 두 축을 다 적용해 보고 근거를 로그에 남긴 뒤 택한다.
- 이름이 `world` 인 빈 루트 링크: 임포터가 `UrdfLink(IsBaseLink)` 만 붙이고 ArticulationBody 는 만들지 않는다. 루트 AB 는 `base_link`(`isRoot`)이고 여기에 `immovable` 을 건다. 문제 없음.
- ROS-TCP-Connector v0.7.0 내장 메시지: `sensor_msgs/JointState` 는 `Runtime/Messages/Sensor/msg/JointStateMsg.cs`(`RosMessageTypes.Sensor.JointStateMsg`) 로 들어 있다. ROS2 모드에서 `header.stamp.sec` 은 int, `header.seq` 는 없다. 커스텀 메시지 생성 불필요(§8 결론과 같음).
- 중복 `<material>`: 파서는 리스트로 받고(중복 검사 없음) `UrdfMaterial.CreateMaterial` 이 같은 이름의 .mat 이 있으면 재사용하므로 첫 정의가 이긴다. 오류·경고 없음. `orange` 는 (1.0, 0.4, 0.0) 채택.
- 덤으로 알게 된 것: `ArticulationBody.maxAngularVelocity` 는 씬에 직렬화되지 않는다. 임포터가 URDF 속도 한계(rad/s)로 넣은 값이 씬 재로드 뒤 기본 50 rad/s 로 돌아온다. 이 값은 몸체 각속도 상한이라 관절 속도 한계와 같지도 않다. 관절 속도 한계는 JointLimitMonitor 가 감시만 한다(README 한계).

---

### 9-5. 단계 7에서 확인한 것 (2026-10-21)

**Isaac 플랜트의 물리 설정**(딸기 리포 읽기 전용, `pxr` 로 USD 를 읽음). Unity 는 중력만 빼고 이 값에 맞췄다(D-4 는 중력 off).

| 항목 | Isaac 값 | 출처 | Unity |
|---|---|---|---|
| 드라이브 강성·감쇠 | 팔 1e6·1e5, 그리퍼 1e5·1e4 (감쇠/강성 비 0.1) | `strawberry_harvest/assets/robot/robot_assembly.usd` @5710f09 `drive:angular` 덮어쓰기 (`robot_physics.usd` 의 원래 값 54–2648 을 덮음) | 같은 값, SimSetup 이 씬에 저장 |
| 드라이브 최대 힘 | 194 / 194 / 194 / 66 / 66 / 66 / 1000×4 (URDF effort) | `configuration/robot_physics.usd` `drive:angular:physics:maxForce` | 같은 값 |
| 관절 최대 속도 | 180 / 225 / 372.4 °/s | `robot_physics.usd` `physxJoint:maxJointVelocity` | `maxJointVelocity`(rad/s), Play 때 건다(씬에 저장 안 됨). 효과는 미확인 |
| 풀이기·반복 | TGS, 아티큘레이션 위치 32·속도 1 | `/physicsScene physxScene:solverType`, `root_joint physxArticulation:solver*IterationCount` | 프로젝트 TGS, 루트 `solverIterations` 32/1(Play 때) |
| 중력 | 켜짐 9.81 | `/physicsScene` | 꺼짐(D-4) |

드라이브만 맞추고 풀이기가 PGS·반복 6이면 관절 사이 반동으로 J2 가 한계(−95°)를 0.003–0.66° 넘었다. 풀이 반복을 Isaac 처럼 32 로 올려야 사라졌다. TGS 32회는 한계도 단단하게 막아서, Mirror 모드에서 받은 한계 밖 값(137°)을 Unity 관절이 따라가지 않는다. 그래서 M1 한계 판정은 Unity 측정값이 아니라 받은 메시지 값으로 한다.

**M2 헤드리스 런**(`scripts/run_m2_headless.sh`, Unity batchmode 플랜트 + 딸기 노드 4개 + 트리거): 루프가 Isaac 없이 닫힌다. 판정 근거는 §6 대로 가상 제어기·스캔 로그다(`logs/m2/*_m2_headless/summary.txt`).

**MoveLine IK 실패의 기준율.** Unity 런에서 직선 진입·줄기 하강 MoveLine 이 가상 제어기 IK 단계에서 실패해 픽 하나를 건너뛰는 일이 있었다(런마다 다른 과실). 딸기 리포에 보존된 Isaac 완주 런 77개 중 24개(31%)에도 같은 `MoveLine IK Failed` 가 있다(픽 596회 중 24회, 4.0%). 가상 제어기의 MoveLine 은 플랜트 측정 자세에서 출발해 5 mm 씩 IK 를 잇기 때문에 도달 경계 근처 과실에서 출발 자세 차이(도착 허용 0.3° 안)에 민감하다.

## 부록 A. 문서와 코드가 다른 곳 (딸기 리포, 참고용)

딸기 리포는 고치지 않는다. Unity 시뮬레이터 작업 중 문서를 잘못 믿지 않도록 적어 둔다.

| 문서 서술 | 코드 사실 |
|---|---|
| `robot.urdf.xacro:12` "world_fixed 를 지웠다" | `rh_p12_rn.xacro:17-21` 에 그대로 있다 |
| `docs/urdf_setup.md` xacro 수정 뒤 `build_urdf.sh` 로 갱신 | 재빌드하면 손편집(D1 한계, 파츠 pitch 0)이 사라진다 |
| `docs/e0509_spec_audit.md:19` "robot.urdf(USD 임포트 원본)" | USD 는 06-18 파일럿 URDF 임포트본이다. 관절 값은 같다 |
| `R:178`, `docs/parameters.md:307` "fake_vision 2Hz" | 입력이 1 Hz 라 실제 약 1 Hz |
| `B:284` 주석 "RELEASE — 그 자리에 정지" | Isaac 은 트레이 안/밖에 따라 동적 낙하시킨다 |
| `docs/run_guide.md:863,963` "--symlink-install 이라 재시작만" | 복사 설치다. src 변경은 재빌드해야 반영된다 |
| `docs/pipeline_overview.md:111-118` 플래너 구독 표 | `/strawberry/detection/pick_pose`·`/sim/grasp_event` 는 플래너가 구독하지 않는다. `/dsr01/curobo/*` 등이 빠져 있다 |
| `README.md:204` "이 노드만 빼면 진짜 두산 드라이버가 같은 서비스 제공" | 두산 motion 서비스 4개는 이름이 같다. `/gripper_service/*` 는 두산 드라이버 서비스가 아니고 실기 제공자 소스가 리포에 없다 |
| `gripper_client.py` 의 SafeGrasp 결과 필드 | `SafeGrasp.action` 에 없는 필드(`grasp_detected`, `object_lost`, `message` 등)를 읽는다. 경로가 꺼져 있어 드러나지 않는다 |
