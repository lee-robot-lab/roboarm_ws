import ast

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from arm_msgs.msg import MitCommand, MitState


def parse_list_param(val, name: str):
    """Accept list/tuple or a string like '[0.8, 0.2, -0.3, 0.0]'."""
    if isinstance(val, (list, tuple)):
        return list(val)
    if isinstance(val, str):
        try:
            parsed = ast.literal_eval(val)
        except Exception as e:
            raise RuntimeError(f"Failed to parse '{name}' from string: {val}") from e
        if not isinstance(parsed, (list, tuple)):
            raise RuntimeError(f"'{name}' must be a list, got: {type(parsed)} from {val}")
        return list(parsed)
    raise RuntimeError(f"'{name}' must be list or string, got: {type(val)}")


class MitHold(Node):
    def __init__(self):
        super().__init__("mit_hold")

        # motor_ids는 launch에서 파이썬 리스트로 줄 거라 기본 list로 둬도 OK
        self.declare_parameter("motor_ids", [1, 2, 3, 4])

        # q_targets는 launch에서 문자열로 넘길 거라 기본도 문자열로 둠
        self.declare_parameter("q_targets", [0.8, 0.0, 0.0, 0.0])

        self.declare_parameter("kp", 30.0)
        self.declare_parameter("kd", 1.0)

        motor_ids_raw = self.get_parameter("motor_ids").value
        q_targets_raw = self.get_parameter("q_targets").value

        self.motor_ids = [int(x) for x in parse_list_param(motor_ids_raw, "motor_ids")]
        self.q_targets = [float(x) for x in parse_list_param(q_targets_raw, "q_targets")]

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)

        if len(self.motor_ids) != len(self.q_targets):
            raise RuntimeError("motor_ids and q_targets must have same length")

        self.target_map = {mid: q for mid, q in zip(self.motor_ids, self.q_targets)}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(MitCommand, "/arm/mit_cmd", qos)
        self.sub_state = self.create_subscription(MitState, "/arm/mit_state", self.on_state, qos)

        self.get_logger().info(f"MitHold multi ready: {self.target_map} | kp={self.kp}, kd={self.kd}")

    def on_state(self, st: MitState):
        mid = int(st.motor_id)
        if mid not in self.target_map:
            return

        now = self.get_clock().now()

        cmd = MitCommand()
        cmd.stamp = now.to_msg()
        cmd.motor_id = mid
        cmd.q_des = float(self.target_map[mid])
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
