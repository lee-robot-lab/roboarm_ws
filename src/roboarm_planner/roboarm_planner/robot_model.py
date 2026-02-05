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
