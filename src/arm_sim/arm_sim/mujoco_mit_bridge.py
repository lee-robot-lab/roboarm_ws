import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from arm_msgs.msg import MitCommand, MitState


class MujocoMitBridge(Node):
    def __init__(self):
        super().__init__("mujoco_mit_bridge")

        self.declare_parameter("xml_path", "robot.xml")
        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_viewer", True)

        # multi-motor mapping
        self.declare_parameter("motor_ids", [1, 2, 3, 4])
        self.declare_parameter("joint_indices", [0, 1, 2, 3])
        self.declare_parameter("actuator_indices", [0, 1, 2, 3])

        xml_path = self.get_parameter("xml_path").value
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

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # latest commands per motor_id
        self.latest_cmd = {}
        for mid in self.motor_ids:
            cmd = MitCommand()
            cmd.motor_id = int(mid)
            cmd.q_des = 0.0
            cmd.qd_des = 0.0
            cmd.kp = 20.0
            cmd.kd = 0.5
            cmd.tau_ff = 0.0
            self.latest_cmd[int(mid)] = cmd

        self.viewer = None
        if self.use_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(1.0 / self.hz, self.step_and_publish)
        self.get_logger().info(
            f"Loaded MJCF: {xml_path} | hz={self.hz} | motors={self.motor_ids} "
            f"| joints={self.joint_indices} | actuators={self.actuator_indices}"
        )

    def on_cmd(self, msg: MitCommand):
        mid = int(msg.motor_id)
        if mid not in self.latest_cmd:
            return
        self.latest_cmd[mid] = msg

    def step_and_publish(self):
        # apply all motor commands
        for mid, jidx, aidx in zip(self.motor_ids, self.joint_indices, self.actuator_indices):
            cmd = self.latest_cmd[int(mid)]
            self.data.ctrl[int(aidx)] = float(cmd.q_des)

        mujoco.mj_step(self.model, self.data)

        now_msg = self.get_clock().now().to_msg()

        # publish all motor states
        for mid, jidx in zip(self.motor_ids, self.joint_indices):
            jidx = int(jidx)

            q = float(self.data.qpos[jidx])
            qd = float(self.data.qvel[jidx])

            tau = 0.0
            try:
                tau = float(self.data.qfrc_actuator[jidx])
            except Exception:
                pass

            out = MitState()
            out.stamp = now_msg
            out.motor_id = int(mid)
            out.q = q
            out.qd = qd
            out.tau = tau
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
