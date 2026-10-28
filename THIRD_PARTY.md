# 제3자 자산 출처·라이선스

이 리포에 들어 있는 파일 가운데 이 프로젝트에서 만들지 않은 자산의 출처와 라이선스를 정리한다(조사 2026-10-22). 근거는 원본 파일, 원본 패키지의 `package.xml`, 상류 저장소의 `LICENSE`, 딸기 리포(`~/strawberry_grasp_environment`)의 git 기록이다. 법률 판단이 아니라, 확인한 사실과 공개 전에 정한 항목을 적는다.

- **이 리포에서 만든 코드·문서·자산은 MIT 라이선스다**([`LICENSE`](LICENSE), © 2026 OceanShape, 2026-10-24 정함). 아래 제3자 자산(1·2·5 의 원래 형상)은 MIT 대상이 아니고 각자의 조건을 따른다.
- 표의 "이 리포에서 바꾼 것"은 이 리포 안의 사본에만 한 일이다. 이번 작업에서 딸기 리포와 상류 저장소 파일 변경 없음.
- 라이선스 원문 사본은 [`third_party_licenses/`](third_party_licenses/)에 있다(§5).

## 1. 자산 목록

| # | 자산 | 이 리포 위치 | 원본 | 라이선스 근거 | 이 리포에서 바꾼 것 |
|---|---|---|---|---|---|
| 1 | 두산 e0509 팔 메쉬 `.dae` 28개(visual 14·collision 14) | `assets_src/e0509/dsr_description2/meshes/`, `UnityProject/Assets/URDF/e0509/dsr_description2/meshes/` | [DoosanRobotics/doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) `ec92425`(2026-03-24) `dsr_description2/meshes/e0509_{white,collision}/`. 딸기 리포 `src/doosan_robot2` 클론에서 복사 | 저장소 `LICENSE`: BSD 3-Clause, "Copyright (c) 2024, Doosan Robotics Inc." · `dsr_description2/package.xml`: `Apache License 2.0`. **두 표기가 다르다** — 둘 다 고지한다 | 20개: 노드 행렬을 정점·법선에 구워 넣음, 파일 맨 위에 수정 고지 주석(§2-3) |
| 2 | ROBOTIS RH-P12-RN 그리퍼 메쉬 `.stl` 5개 | `assets_src/e0509/rh_p12_rn_description/meshes/`, `UnityProject/Assets/URDF/e0509/rh_p12_rn_description/meshes/` | [ROBOTIS-GIT/RH-P12-RN](https://github.com/ROBOTIS-GIT/RH-P12-RN) `58d06e7`(2020-02-17) `rh_p12_rn_description/meshes/`. 딸기 리포 `src/RH-P12-RN` 클론에서 복사 | Apache License 2.0(저장소 `LICENSE`·`package.xml` 같음, NOTICE 파일 없음). 저작권 표기는 소스 머리말의 "Copyright 2018 ROBOTIS CO., LTD." | 바꾸지 않고 복사 |
| 3 | 그리퍼 부품 메쉬 `gripper_parts.stl` | `assets_src/e0509/strawberry_grasp_environment_description/meshes/`, Unity 사본 | 딸기 리포 `src/strawberry_grasp_environment_description`(이전 프로젝트) | `package.xml`: `TODO: License declaration`(미기재) | 바꾸지 않고 복사 |
| 4 | URDF `e0509.urdf` | `assets_src/e0509/`, `UnityProject/Assets/URDF/e0509/` | 딸기 리포 `robot.urdf`(HEAD `2797bb5`, 파일 커밋 `a2e9bb5`). 1·2 의 xacro 와 3 으로 만든 파일 | 1·2·3 을 따른다 | 메쉬 경로 → `package://`, 중복 `world` 링크 삭제, 그리퍼 상한 1.0 → 1.1 rad, ros2_control·transmission·gazebo 블록 삭제(CONTRACT §1-4) |
| 5 | RealSense D455 본체 자리: `scene.json` 의 `d455_*` 메쉬 8개 | `assets_src/scene/scene.json`, `UnityProject/Assets/Environment/Meshes/d455_*.asset` | NVIDIA Isaac Sim 자산 `Isaac/Sensors/Intel/RealSense/rsd455.usd`. 딸기 리포 `strawberry_harvest/assets/omniverse_imports/` 의 사본에서 `scripts/export_isaac_scene.py` 로 부품 크기·자리를 읽음 | NVIDIA Omniverse License Agreement(§2-1) | **형상을 넣지 않는다**: 부품 8개를 각자의 축정렬 경계 상자(삼각형 12개씩, 모두 96개)로 바꿨다(2026-10-22, `bbox_box`). 상자는 크기·자리만 쓴 이 리포의 도형이라 MIT 로 둔다. 원래 형상은 11,846 삼각형. 재질은 색·거칠기 값만(NVIDIA 텍스처·MDL 은 가져오지 않음) |
| 6 | 딸기 모델: `strawberry` 메쉬(6,770 삼각형) + 텍스처 3장 | `assets_src/scene/scene.json`, `assets_src/scene/textures/strawberry_*.png`, `UnityProject/Assets/Environment/Textures/` | 딸기 리포 `strawberry_harvest/assets/strawberry/`. 메쉬 `strawberry_ripe.usd`·`strawberry_unripe.usd` 는 "Blender v5.1.2" 로 내보낸 파일, 텍스처 `frut333.png`·`frut333_Normal.png`·`frut_2333_Normal.png`. 딸기 리포 첫 커밋(`4697cf4`, 2026-06-18)부터 있다 | **직접 제작** — 부트캠프 때 실험실의 딸기를 보고 치수를 재서 만든 모델. 파일·커밋에는 제작자 기록이 없다. 이 리포 라이선스(MIT)(§2-2) | 텍스처를 1024 px 로 줄임, 좌표 변환 |
| 7 | 책상 상판 `table_top` 메쉬(408 삼각형) | `assets_src/scene/scene.json` | 딸기 리포 `strawberry_harvest/assets/props/table.usd`("Blender v5.1.2", 2026-06-18 첫 커밋부터) | **직접 제작** — 실험실 책상을 실측해 만든 모델. 이 리포 라이선스(MIT)(§2-2) | 좌표 변환 |
| 8 | 계란판(7,172 삼각형)·꼭지(388 삼각형) 메쉬 | `assets_src/scene/scene.json` | 딸기 리포에서 스크립트로 만든 메쉬 — 계란판 `d6cba0f`·`639897c`(2026-09-11), 꼭지 `717b5f0`(2026-09-10) | 이전 프로젝트(딸기 리포, 라이선스 미기재) | 좌표 변환 |
| 9 | 보드 격자 텍스처 `board_albedo.png` | `assets_src/scene/textures/`, `UnityProject/Assets/Environment/Textures/` | 딸기 리포 `strawberry_harvest/assets/props/textures/paper_grid_custom_bg.png`(테두리·중심 십자만 있는 격자, 2026-06-18 부터) | **직접 제작** — 이전 프로젝트(딸기 리포)에서 직접 그린 격자 그림. 이 리포 라이선스(MIT)(§2-2) | 크기 조정 |
| 10 | 보드 방위 글자 PNG 9장 | `assets_src/scene/textures/board_labels/`, `UnityProject/Assets/Environment/Textures/board_labels/` | 딸기 리포 `strawberry_harvest/scripts/scene_tools/gen_board_label_textures.py`(`2f3174a`, 2026-09-30)가 그린 그림. 글꼴은 그 스크립트의 첫 후보로 이 PC 에 있는 Isaac Sim 설치본의 `kit/resources/fonts/OpenSans-SemiBold.ttf` | 그림은 이전 프로젝트에서 만든 것. 글꼴 파일은 이 리포에 없다(설치본 안에 그 글꼴의 라이선스 파일은 같이 들어 있지 않다) | 바꾸지 않고 복사 |
| 11 | HUD 라벨 PNG 45장 + `manifest.json` | `assets_src/hud/labels/`, `UnityProject/Assets/Resources/HudLabels/` | 딸기 리포 `strawberry_harvest/scripts/hud/make_labels.py` 가 Noto Sans CJK(`/usr/share/fonts/opentype/noto/NotoSansCJK-{Regular,Bold}.ttc`)로 그린 그림 | 그림은 이전 프로젝트에서 만든 것. 글꼴은 SIL OFL 1.1(데비안 `fonts-noto-cjk` 의 copyright 파일), 글꼴 파일은 이 리포에 없다 | 바꾸지 않고 복사 |

이 리포에서 만든 단순 도형은 고지 대상이 아니다: 보드 사각형, 보드 하이라이트 테두리 20개, 책상 다리·받침판·바닥 상자(딸기 씬에서는 크기·위치만 가져왔다).

## 2. 공개 전에 정한 것

### 2-1. D455 형상 (NVIDIA 자산)

- 근거는 이 PC 의 Isaac Sim 설치본 `isaacsim/LICENSE.txt`(NVIDIA Omniverse License Agreement, 최신판: <https://docs.omniverse.nvidia.com/dev-guide/latest/common/NVIDIA_Omniverse_License_Agreement.html>)다.
  - §1.4 가 NVIDIA 3D 자산을 "Content" 로 정의한다.
  - §2.1 은 Content 를 (수정했더라도) 자기 제품·작업물에 넣어 배포하는 것은 허용하고, Content 만 따로 배포하는 것은 허용하지 않는다.
  - §3.2 는 배포 조건이 이 라이선스만큼 보호적이어야 한다고 한다. 나중에 이 리포에 관대한 라이선스를 붙이면 D455 형상은 그 대상에서 빼야 한다는 뜻으로 읽힌다.
- 이 PC 가 자산 묶음을 받은 경로의 조건이 설치본 라이선스와 다를 수 있다. 받은 경로의 조건도 확인한다.
- 선택지는 (가) 그대로 두고 NVIDIA 조건을 따른다고 밝히기, (나) 단순 상자로 바꾸기였다. **(나)로 정해 바꿨다(2026-10-22)**: 지금 파일에는 부품마다 경계 상자(크기·자리)만 있고 NVIDIA 형상은 없다.
  - 화면: 렌더 15장을 바꾸기 전과 비교해 달라진 것은 D455 자리와 그 그림자뿐이고, 손목 카메라 화면은 픽셀까지 같다(본체는 손목 카메라에서 빠진다). 시험(화면·Command 4종, M1 스모크, M2 헤드리스)은 그대로 통과(PROGRESS 2026-10-22).
- **git 기록**: 원래 형상(`assets_src/scene/scene.json`, `UnityProject/Assets/Environment/Meshes/d455_*.asset`)은 작업 리포의 커밋 `2b01433`(2026-10-21)부터 `1d818a7`(2026-10-22)까지의 기록에 남아 있다. 그래서 공개 리포는 작업 리포를 그대로 올리지 않고 지금 파일만 담은 기록 없는 새 리포로 만들었다 — 공개 리포에는 원래 형상이 들어 있지 않다.
- `docs/media/m2_run_30s.mp4`·`m2_run_poster.jpg` 는 상자로 바꾸기 전에 찍어 원래 D455 모양이 렌더로 보인다(형상 파일은 아니다). **다시 찍지 않고 그대로 둔다(2026-10-28 정함)** — 화면 녹화이고, 형상을 상자로 바꾼 사정은 이 절과 `docs/DETAILS.md` 에 적었다.
- 딸기 리포에도 `rsd455.usd` 전체와 NVIDIA MDL·텍스처 사본(`omniverse_imports/Isaac/Materials/`)이 있다. 그 리포를 공개하면 같은 질문이 생긴다.

### 2-2. 딸기 모델·책상 상판·보드 격자의 출처

- 6(딸기 모델)·7(책상 상판)은 **정함(2026-10-24)**: 부트캠프 때 실험실의 딸기·책상을 보고 치수를 재서 직접 만든 모델이다. 이 리포 라이선스(MIT)로 둔다.
- 9(보드 격자 텍스처)도 **정함(2026-10-28)**: 이전 프로젝트에서 직접 그린 격자 그림이다(파일 이름의 `custom`). 이 리포 라이선스(MIT)로 둔다.

### 2-3. 수정한 두산 메쉬의 파일 안 고지 (Apache 2.0 §4(b))

- `package.xml` 표기(Apache 2.0)를 따르면 수정한 파일마다 바꿨다는 눈에 띄는 고지가 있어야 한다(§4(b)). 지금은 이 문서에만 적었고 파일 안에는 고지가 없다. 저장소 `LICENSE`(BSD 3-Clause)에는 이런 요구가 없다.
- 수정한 20개(`e0509_white`·`e0509_collision` 에 같은 이름으로 10개씩): `E0509_1_1`, `1_2`, `1_3`, `1_4`, `2_1`, `2_2`, `3_2`, `4_1`, `4_2`, `6_1`.
  - 바꾼 것: 노드 `<matrix>` 를 정점·법선에 곱해 넣고 노드 행렬을 단위행렬로 했다(`scripts/bake_dae_node_transforms.py`, 기록 `logs/step4/bake_dae.json`).
  - 이유: Unity 의 Collada 임포트가 노드 이동을 정점과 다른 축으로 합성해 link_4·link_6 시각이 약 1.1 m 어긋났다(PROGRESS 단계 4, README 한계).
- 바꾸지 않은 8개(폴더마다 4개): `E0509_0_0`, `3_1`, `5_1`, `5_2`.
- **넣었다(2026-10-24)**: 바꾼 20개의 XML 선언 바로 아래에 영어 주석으로 수정 사실(OceanShape, e0509_unity_sim), 원본 경로(DoosanRobotics/doosan-robot2 `dsr_description2/meshes/…`), 바꾼 내용, 상류 라이선스 위치를 적었다. `bake_dae_node_transforms.py` 가 베이크할 때 함께 쓰므로 `make_unity_urdf.py` 로 다시 만들어도 남는다. 바꾸지 않은 8개에는 넣지 않는다(원본과 바이트가 같다).

## 3. 파생 파일

- Unity 가 URDF 임포트 때 만든 `UnityProject/Assets/URDF/e0509/Materials/*.mat` 12개(URDF `<material>` 색)와 STL 메쉬의 `.prefab`·`.asset` 12개는 1·2·3 에서 나온 파생물이다.
- `docs/media/m2_run_30s.mp4`·`m2_run_poster.jpg` 와 `logs/render/*.png` 는 이 리포의 Unity 화면을 찍은 것이고, 1·2·6·7 의 형상이 보인다. 5(D455)는 `logs/render/` 에서는 상자로, 영상·포스터에서는 원래 모양으로 보인다(§2-1).

## 4. 리포에 넣지 않고 참조만 하는 것

| 이름 | 어떻게 쓰나 | 라이선스 |
|---|---|---|
| Unity URDF-Importer v0.5.2, ROS-TCP-Connector v0.7.0 | `UnityProject/Packages/manifest.json` 의 git URL. 프로젝트를 처음 열 때 내려받는다(`Library/`, git 제외) | 로컬 캐시의 패키지 경로에는 라이선스 파일·표기가 없다 — 상류 저장소에서 확인 |
| ROS-TCP-Endpoint `main-ros2` `54c1a64` | `ros/src/` 에 클론(git 제외) | Apache License 2.0(클론의 `LICENSE`) |
| Unity Recorder 4.0.3, Timeline 1.7.7, Test Framework 1.1.33, Editor Coroutines 1.0.0 | manifest 의 버전. Unity 레지스트리(Recorder 는 에디터 내장 tgz) | Unity Companion License 등(각 패키지의 `LICENSE.md`) |
| 딸기 리포 노드·HUD 모듈 | 실행 때 패키지·모듈 이름으로 참조만 | 딸기 리포 조건(미기재) |

## 5. 라이선스 원문 사본

| 파일 | 원본 | sha256(앞 16자) |
|---|---|---|
| [`third_party_licenses/doosan-robot2_BSD-3-Clause.txt`](third_party_licenses/doosan-robot2_BSD-3-Clause.txt) | doosan-robot2 `ec92425` 의 `LICENSE` | `f86b47192565a2d4`(원본과 같음) |
| [`third_party_licenses/RH-P12-RN_Apache-2.0.txt`](third_party_licenses/RH-P12-RN_Apache-2.0.txt) | RH-P12-RN `58d06e7` 의 `LICENSE` | `c71d239df91726fc`(원본과 같음) |

- `dsr_description2/package.xml` 이 가리키는 Apache License 2.0 원문은 RH-P12-RN 사본과 같은 라이선스다.
- NVIDIA Omniverse License Agreement 는 사본을 두지 않고 §2-1 의 링크로 가리킨다.
