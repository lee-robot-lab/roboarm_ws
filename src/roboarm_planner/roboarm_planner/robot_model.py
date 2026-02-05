"""
robot_model.py

[역할]
- URDF로부터 로봇 모델을 로딩하고, 기구학 계산(FK/Jacobian)에 필요한 공통 기능을 제공한다.
- IK/trajectory/ROS 노드가 Pinocchio 호출 및 URDF 세부사항에 직접 의존하지 않도록 분리한다.

[기준 프레임]
- base 프레임 기준으로 end-effector(ee_link)의 위치를 계산한다.

[주요 기능]
- fk_pos(q): 관절각 q에서 ee_link 위치 (x, y, z) 반환
- (선택) jacobian(q): 특이점 판단/최적화에 필요한 자코비안 반환 가능

[연관 파일]
- ik.py: fk_pos를 사용해 IK 최적화 수행
- plan_node.py: 목표점(/goal_point)을 받아 IK/trajectory 생성
"""

import numpy as np
import pinocchio as pin

class RobotModel:
    def __init__(self, urdf_path: str, ee_frame: str = "ee_link"):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.ee_fid = self.model.getFrameId(ee_frame)
        if self.ee_fid >= len(self.model.frames):
            raise ValueError(f"End-effector frame '{ee_frame}' not found in URDF.")

        self.q_min = self.model.lowerPositionLimit.copy()
        self.q_max = self.model.upperPositionLimit.copy()

    def fk_pos(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_fid]   # WORLD(=base로 가정) 기준
        return oMf.translation.copy()
