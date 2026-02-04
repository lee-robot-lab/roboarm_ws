# roboarm_ws
# RoboArm ROS2 Workspace (MuJoCo + MIT mode + Gravity Compensation)

이 워크스페이스는 **MIT 모드(τ = PD + τ_ff)** 형태의 제어 인터페이스를 ROS2 토픽으로 통일하고,  
**MuJoCo 시뮬레이션 ↔ 컨트롤(Pinocchio 중력보상) ↔ 추후 실기기(CAN)** 를 동일한 메시지로 갈아끼우는 구조를 목표로 합니다.

---

## Package Overview

### `arm_msgs`
**공통 인터페이스 메시지 패키지**
- `MitCommand` : 모터 명령 (MIT mode)
  - `motor_id, q_des, qd_des, kp, kd, tau_ff, stamp ...`
- `MitState` : 모터 상태
  - `motor_id, q, qd, tau, stamp ...`

> 시뮬/컨트롤/실기기 모두 동일한 토픽 타입을 사용합니다.

---

### `arm_sim`
**MuJoCo 플랜트 + ROS 브릿지**
- `robot.xml` (MJCF)
  - 로봇 모델/관성/충돌 포함
  - actuator를 `<position>` 서보에서 `<motor>`(토크 입력)로 변경
  - 정격/피크 토크 범위는 `ctrlrange/forcerange`로 제한
- `mujoco_mit_bridge.py`
  - Subscribe: `/arm/mit_cmd` (`MitCommand`)
  - Publish: `/arm/mit_state` (`MitState`)
  - MIT 토크식으로 `data.ctrl[aidx]`(토크 입력) 생성:


\tau = \tau_{ff} + k_p(q_{des}-q) + k_d(\dot q_{des} - \dot q)

---

### `arm_control`
**컨트롤 노드**
- `mit_hold.py`
  - 테스트용 목표각 홀드(PD)
  - `/arm/mit_cmd` publish
- `gravity_comp_controller.py`
  - Pinocchio 기반 중력항 \(g(q)\) 계산 → `tau_ff`로 출력
  - PD + 중력보상 결합으로 목표각 추종/자세 유지

> ⚠️ 개념 정리  
> - `kp=kd=0`이면 “목표각으로 이동”은 불가능합니다. (토크 = τ_ff만 남음)  
> - 목표 이동은 PD가 담당, 중력 상쇄는 `tau_ff`가 담당합니다.

---

### `arm_bringup`
**실행 구성(launch)**
- 보통 “플랜트(MuJoCo)”와 “컨트롤러”를 분리 실행합니다.
- `/arm/mit_cmd` 충돌 방지를 위해 컨트롤러는 1개만 켭니다.

---

## Topic Graph

- `arm_sim/mujoco_mit_bridge`
  - pub: `/arm/mit_state`
  - sub: `/arm/mit_cmd`

- `arm_control/*` (컨트롤러들)
  - sub: `/arm/mit_state`
  - pub: `/arm/mit_cmd`

> ⚠️ `/arm/mit_cmd`는 **publisher 1개만** 유지해야 합니다.  
> (컨트롤러 2개가 동시에 publish하면 명령이 섞여 충돌합니다.)

---

## Quick Start (Clone → Build → Run)

---

### 0) Clone (GitHub에서 워크스페이스 받기)
```bash
cd ~
git clone git@github.com:lee-robot-lab/roboarm_ws.git
cd roboarm_ws
```


### 1) 의존성 설치
```bash
sudo apt update
rosdep update
cd ~/roboarm_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2) Build
```bash
source /opt/ros/humble/setup.bash
cd ~/roboarm_ws
colcon build
```
### 3) 실행: MuJoCo 시뮬 + 컨트롤러 (터미널 2개)


#### Terminal 1) MuJoCo 플랜트 실행 (시뮬레이터)
```bash
source /opt/ros/humble/setup.bash
source ~/roboarm_ws/install/setup.bash
ros2 launch arm_bringup mujoco_only.launch.py
```
#### Terminal 2) 중력보상 + 목표각 추종 실행
```bashsource /opt/ros/humble/setup.bash
source ~/roboarm_ws/install/setup.bash

ros2 run arm_control gravity_comp_controller
```
