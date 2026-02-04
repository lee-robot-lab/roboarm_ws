import numpy as np
import rclpy
import os
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import pinocchio as pin

from arm_msgs.msg import MitCommand, MitState


class GravityCompController(Node):
    def __init__(self):
        super().__init__("gravity_comp_controller")

        # 파라미터
        self.declare_parameter("motor_ids", [1, 2, 3, 4])
        self.declare_parameter("joint_names", ["j1", "j2", "j3", "j4"])
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("kp", 30.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("q_targets", [0.0, 1.57, 0.0, 0.0])
        self.declare_parameter("signs", [1.0, 1.0, 1.0, 1.0])  # 방향 반전 필요하면 -1

        self.motor_ids = [int(x) for x in self.get_parameter("motor_ids").value]
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.q_targets = [float(x) for x in self.get_parameter("q_targets").value]
        self.signs = np.array(list(self.get_parameter("signs").value), dtype=float)

        urdf_path = str(self.get_parameter("urdf_path").value).strip()

        if not urdf_path:
            # ROS 표준: install/share에서 자동 탐색
            share = get_package_share_directory("arm_sim")
            urdf_path = os.path.join(share, "urdf", "robot.urdf")

        if not os.path.isfile(urdf_path):
            raise RuntimeError(f"URDF not found: {urdf_path}")

        self.get_logger().info(f"Using URDF: {urdf_path}")

        # Pinocchio 모델 로드
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        if len(self.motor_ids) != len(self.joint_names):
            raise RuntimeError("motor_ids 길이와 joint_names 길이가 같아야 함")
        if len(self.q_targets) != len(self.motor_ids):
            raise RuntimeError("q_targets 길이와 motor_ids 길이가 같아야 함")

        # joint_name -> q index 매핑 (순서 가정 X)
        self.mid_to_qidx = {}
        for mid, jn in zip(self.motor_ids, self.joint_names):
            jid = self.model.getJointId(jn)
            if jid == 0:
                raise RuntimeError(f"URDF에 joint '{jn}'을 못 찾음")
            qidx = int(self.model.idx_qs[jid])
            self.mid_to_qidx[mid] = qidx

        # 최신 state 저장
        self.last_q = {mid: 0.0 for mid in self.motor_ids}
        self.last_qd = {mid: 0.0 for mid in self.motor_ids}
        self.have = {mid: False for mid in self.motor_ids}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(MitState, "/arm/mit_state", self.on_state, qos)
        self.pub = self.create_publisher(MitCommand, "/arm/mit_cmd", qos)

        # 컨트롤 루프 주기(시뮬 hz에 맞추기)
        self.timer = self.create_timer(1.0 / 200.0, self.step)

        self.get_logger().info(
            f"GravityComp ready | urdf={urdf_path} | map(mid->qidx)={self.mid_to_qidx}"
        )

    def on_state(self, msg: MitState):
        mid = int(msg.motor_id)
        if mid not in self.have:
            return
        self.have[mid] = True
        self.last_q[mid] = float(msg.q)
        self.last_qd[mid] = float(msg.qd)

    def step(self):
        if not all(self.have.values()):
            return

        # Pinocchio q, dq 벡터 구성 (nq=4일 것)
        q = np.zeros(self.model.nq, dtype=float)
        dq = np.zeros(self.model.nv, dtype=float)

        for mid in self.motor_ids:
            qidx = self.mid_to_qidx[mid]
            q[qidx] = self.last_q[mid]
            dq[qidx] = self.last_qd[mid]

        # 중력항 g(q)
        tau_g = pin.computeGeneralizedGravity(self.model, self.data, q)
        
        # 모터별 cmd 발행
        for i, mid in enumerate(self.motor_ids):
            qidx = self.mid_to_qidx[mid]

            cmd = MitCommand()
            cmd.stamp = self.get_clock().now().to_msg()
            cmd.motor_id = int(mid)

            cmd.q_des = float(self.q_targets[i])
            cmd.qd_des = 0.0
            cmd.kp = float(self.kp)
            cmd.kd = float(self.kd)

            cmd.tau_ff = float(self.signs[i] * tau_g[qidx])
            self.pub.publish(cmd)


def main():
    rclpy.init()
    node = GravityCompController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
