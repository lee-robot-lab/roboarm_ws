import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from arm_msgs.msg import MitCommand, MitState


class MitHold(Node):
    def __init__(self):
        super().__init__("mit_hold")

        self.declare_parameter("motor_id", 1)
        self.declare_parameter("q_target", 0.8)
        self.declare_parameter("kp", 30.0)
        self.declare_parameter("kd", 1.0)

        self.motor_id = int(self.get_parameter("motor_id").value)
        self.q_target = float(self.get_parameter("q_target").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_cmd = self.create_publisher(MitCommand, "/arm/mit_cmd", qos)
        self.sub_state = self.create_subscription(MitState, "/arm/mit_state", self.on_state, qos)

        self.get_logger().info(f"MitHold ready: motor_id={self.motor_id}, q_target={self.q_target}")

    def on_state(self, st: MitState):
        if st.motor_id != self.motor_id:
            return

        now = self.get_clock().now()

        cmd = MitCommand()
        cmd.stamp = now.to_msg()
        cmd.motor_id = self.motor_id
        cmd.q_des = self.q_target
        cmd.qd_des = 0.0
        cmd.kp = self.kp
        cmd.kd = self.kd
        cmd.tau_ff = 0.0

        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = MitHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
