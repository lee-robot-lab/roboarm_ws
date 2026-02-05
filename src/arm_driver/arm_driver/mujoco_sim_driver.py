import mujoco
import mujoco.viewer
import os  # 추가됨

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory # 추가됨

from arm_msgs.msg import MitCommand, MitState

class MujocoMitBridge(Node):
    def __init__(self):
        super().__init__("mujoco_sim_driver")

        self.declare_parameter("xml_path", "robot.xml")
        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_viewer", True)

        # multi-motor mapping
        self.declare_parameter("motor_ids", [1, 2, 3, 4])
        self.declare_parameter("joint_indices", [0, 1, 2, 3])
        self.declare_parameter("actuator_indices", [0, 1, 2, 3])

        xml_path_param = self.get_parameter("xml_path").value
        self.hz = float(self.get_parameter("publish_hz").value)
        self.use_viewer = bool(self.get_parameter("use_viewer").value)

        self.motor_ids = list(self.get_parameter("motor_ids").value)
        self.joint_indices = list(self.get_parameter("joint_indices").value)
        self.actuator_indices = list(self.get_parameter("actuator_indices").value)

        if not (len(self.motor_ids) == len(self.joint_indices) == len(self.actuator_indices)):
            raise RuntimeError("motor_ids/joint_indices/actuator_indices must have same length")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_state = self.create_publisher(MitState, "/arm/mit_state", qos)
        self.sub_cmd = self.create_subscription(MitCommand, "/arm/mit_cmd", self.on_cmd, qos)

        # [핵심 수정] XML 파일의 경로 문제를 해결하는 로직
        self.get_logger().info(f"Loading XML from: {xml_path_param}")
        
        # 1. XML 파일을 문자열로 읽어옵니다.
        with open(xml_path_param, 'r') as f:
            xml_content = f.read()

        # 2. 'arm_description' 패키지의 실제 설치 경로를 찾습니다.
        try:
            pkg_path = get_package_share_directory('arm_description')
            mesh_abs_path = os.path.join(pkg_path, 'meshes')
            
            # 3. XML 안의 상대 경로(../arm_description/meshes)를 절대 경로로 교체합니다.
            #    (robot.xml에 적어둔 경로 문자열과 정확히 일치해야 합니다)
            target_str = "../arm_description/meshes"
            if target_str in xml_content:
                self.get_logger().info(f"Replacing '{target_str}' with '{mesh_abs_path}'")
                xml_content = xml_content.replace(target_str, mesh_abs_path)
            else:
                self.get_logger().warn(f"'{target_str}' not found in XML. Mesh loading might fail.")

        except Exception as e:
            self.get_logger().error(f"Failed to resolve mesh path: {e}")

        # 4. 수정된 내용(문자열)으로 MuJoCo 모델을 로드합니다.
        self.model = mujoco.MjModel.from_xml_string(xml_content)
        self.data = mujoco.MjData(self.model)

        # latest commands per motor_id
        self.latest_cmd = {}
        for mid in self.motor_ids:
            cmd = MitCommand()
            cmd.motor_id = int(mid)
            cmd.q_des = 0.0
            cmd.qd_des = 0.0
            cmd.kp = 10.0 # 안전을 위해 기본 게인 약간 낮춤
            cmd.kd = 0.5
            cmd.tau_ff = 0.0
            self.latest_cmd[int(mid)] = cmd

        self.viewer = None
        if self.use_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(1.0 / self.hz, self.step_and_publish)
        self.get_logger().info("Simulation Started Successfully!")

    def on_cmd(self, msg: MitCommand):
        mid = int(msg.motor_id)
        if mid not in self.latest_cmd:
            return
        self.latest_cmd[mid] = msg

    def step_and_publish(self):
        # apply all motor commands (MIT-style torque)
        for mid, jidx, aidx in zip(self.motor_ids, self.joint_indices, self.actuator_indices):
            mid = int(mid)
            jidx = int(jidx)
            aidx = int(aidx)

            cmd = self.latest_cmd[mid]

            q  = float(self.data.qpos[jidx])
            qd = float(self.data.qvel[jidx])

            tau = float(cmd.tau_ff) \
                + float(cmd.kp) * (float(cmd.q_des) - q) \
                + float(cmd.kd) * (float(cmd.qd_des) - qd)

            # safety: NaN/inf 방지
            if not math.isfinite(tau):
                tau = 0.0

            # optional: MJCF ctrlrange로 한번 더 클램프(정격 제한 유지)
            try:
                lo, hi = self.model.actuator_ctrlrange[aidx]
                if self.model.actuator_ctrllimited[aidx]:
                    tau = max(lo, min(hi, tau))
            except Exception:
                pass

            self.data.ctrl[aidx] = tau

        mujoco.mj_step(self.model, self.data)

        now_msg = self.get_clock().now().to_msg()

        # publish all motor states
        for mid, jidx, aidx in zip(self.motor_ids, self.joint_indices, self.actuator_indices):
            mid = int(mid); jidx = int(jidx); aidx = int(aidx)

            q  = float(self.data.qpos[jidx])
            qd = float(self.data.qvel[jidx])

            # actuator가 실제로 낸 토크
            tau_meas = 0.0
            try:
                tau_meas = float(self.data.actuator_force[aidx])
            except Exception:
                try:
                    tau_meas = float(self.data.qfrc_actuator[jidx])
                except Exception:
                    pass

            out = MitState()
            out.stamp = now_msg
            out.motor_id = mid
            out.q = q
            out.qd = qd
            out.tau = tau_meas
            out.temp_c = 0.0
            out.error_code = 0
            self.pub_state.publish(out)

        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None

def main():
    rclpy.init()
    node = MujocoMitBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()