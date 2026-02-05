# RoboArm ROS2 Workspace (`roboarm_ws`)

ROS 2 Humble 기반의 4축 로보암 워크스페이스입니다.  
핵심 목표는 **MIT 모드 토픽 인터페이스(`arm_msgs`)를 중심으로** 시뮬레이터/컨트롤러/플래너를 느슨하게 결합하는 것입니다.

---

## 1) 패키지 구성

- `arm_msgs`: 공통 메시지 (`MitCommand`, `MitState`)
- `arm_description`: URDF + mesh 리소스
- `arm_sim`: MuJoCo XML 리소스 패키지 (`robot.xml`)
- `arm_driver`: MuJoCo 시뮬레이션 드라이버 노드 (`mujoco_sim_driver`)
- `arm_control`: 제어 노드 (`gravity_comp_controller`)
- `roboarm_planner`: IK/trajectory/bridge/planner 노드
- `arm_bringup`: launch 모음

파일 인덱스: `docs/FILE_INDEX.md`

---

## 2) 실행 구조

### Topic
- 드라이버(`arm_driver/mujoco_sim_driver`)
  - Sub: `/arm/mit_cmd` (`arm_msgs/MitCommand`)
  - Pub: `/arm/mit_state` (`arm_msgs/MitState`)

- 컨트롤러(`arm_control/gravity_comp_controller`)
  - Sub: `/arm/mit_state`
  - Pub: `/arm/mit_cmd`

> `/arm/mit_cmd` 퍼블리셔는 동시에 1개만 실행하세요.

### Launch
- 기본 실행: `arm_bringup/launch/mujoco.launch.py`
  - 내부에서 `arm_driver/mujoco_sim_driver`를 실행합니다.

---

## 3) 코드 실행법 (정리본)

## 3-1) 최초 1회: 의존성 + 빌드

```bash
cd ~/roboarm_ws
source /opt/ros/humble/setup.bash

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build
```

---

## 3-2) 실행 전 공통 준비 (매 터미널)

모든 터미널에서 아래를 먼저 실행:

```bash
source /opt/ros/humble/setup.bash
source ~/roboarm_ws/install/setup.bash
```

---

## 3-3) 기본 실행 (권장)

### Terminal 1: MuJoCo 드라이버

```bash
ros2 launch arm_bringup mujoco.launch.py
```

필요 시 파라미터 변경:

```bash
ros2 launch arm_bringup mujoco.launch.py publish_hz:=200.0
```

### Terminal 2: 중력보상 컨트롤러

```bash
ros2 run arm_control gravity_comp_controller
```

필요 시 파라미터 변경 예시:

```bash
ros2 run arm_control gravity_comp_controller --ros-args \
  -p kp:=30.0 -p kd:=1.0 \
  -p q_targets:='[0.0, 1.57, 0.0, 0.0]'
```

---

## 3-4) 실행 확인 커맨드

### 토픽 연결 확인

```bash
ros2 topic list
ros2 topic info /arm/mit_cmd
ros2 topic info /arm/mit_state
```

### 상태값 샘플 확인

```bash
ros2 topic echo /arm/mit_state --once
```

---

## 4) 유지보수 팁

- 파일 인덱스 갱신:
```bash
python3 tools/repo_inventory.py
```
- launch/README를 함께 수정해 실제 실행 경로와 문서를 항상 일치시키세요.
