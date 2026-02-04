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
        self.declare_parameter("motor_id", 1)
        self.declare_parameter("joint_index", 0)     # j1 -> 0
        self.declare_parameter("actuator_index", 0)  # a_j1 -> 0
        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("use_viewer", True)

        xml_path = self.get_parameter("xml_path").value
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.jidx = int(self.get_parameter("joint_index").value)
        self.aidx = int(self.get_parameter("actuator_index").value)
        self.hz = float(self.get_parameter("publish_hz").value)
        self.use_viewer = bool(self.get_parameter("use_viewer").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_state = self.create_publisher(MitState, "/arm/mit_state", qos)
        self.sub_cmd = self.create_subscription(MitCommand, "/arm/mit_cmd", self.on_cmd, qos)

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # latest command
        self.latest_cmd = MitCommand()
        self.latest_cmd.motor_id = self.motor_id
        self.latest_cmd.q_des = 0.0
        self.latest_cmd.qd_des = 0.0
        self.latest_cmd.kp = 20.0
        self.latest_cmd.kd = 0.5
        self.latest_cmd.tau_ff = 0.0

        self.viewer = None
        if self.use_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(1.0 / self.hz, self.step_and_publish)
        self.get_logger().info(f"Loaded MJCF: {xml_path} | hz={self.hz} | jidx={self.jidx} | aidx={self.aidx}")

    def on_cmd(self, msg: MitCommand):
        if msg.motor_id != self.motor_id:
            return
        self.latest_cmd = msg

    def step_and_publish(self):
        # Apply q_des to position actuator
        self.data.ctrl[self.aidx] = float(self.latest_cmd.q_des)

        mujoco.mj_step(self.model, self.data)

        q = float(self.data.qpos[self.jidx])
        qd = float(self.data.qvel[self.jidx])

        # Torque estimate (optional)
        tau = 0.0
        try:
            tau = float(self.data.qfrc_actuator[self.jidx])
        except Exception:
            pass

        out = MitState()
        out.stamp = self.get_clock().now().to_msg()
        out.motor_id = self.motor_id
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
