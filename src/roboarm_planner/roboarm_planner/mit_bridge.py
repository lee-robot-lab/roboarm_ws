"""
mit_bridge.py  (ROS2 node: mit_bridge)

[역할]
- MuJoCo MIT 인터페이스와 일반 ROS 토픽(/joint_states, /joint_trajectory)을 연결하는 실행/브릿지 노드.

[구독(Subscribe)]
- /arm/mit_state (arm_msgs/msg/MitState)
  - MuJoCo가 모터 단위 상태를 발행 (motor_id, q, qd, tau, ...)
  - 퍼블리셔 QoS가 BEST_EFFORT인 경우가 있어, 구독 QoS를 sensor_data로 맞춰야 수신 가능

- /joint_trajectory (trajectory_msgs/msg/JointTrajectory)
  - ik_planner_node가 생성한 trajectory
  - 이를 시간에 따라 샘플링하여 모터 명령으로 변환

[발행(Publish)]
- /joint_states (sensor_msgs/msg/JointState)
  - 모터별 상태를 4축(j1~j4)으로 묶어 publish (플래너/기타 노드 호환용)

- /arm/mit_cmd (arm_msgs/msg/MitCommand)
  - motor_id, q_des, qd_des, kp, kd, tau_ff를 모터 단위로 스트리밍 발행
  - mujoco_mit_bridge가 이 토픽을 구독하여 로봇을 실제로 움직인다.

[주의]
- /joint_trajectory는 기본이 VOLATILE이라, mit_bridge가 먼저 떠 있어야 trajectory를 놓치지 않는다.
  (개선 가능) 플래너 publisher를 TRANSIENT_LOCAL로 변경하면 늦게 떠도 마지막 trajectory를 받을 수 있음.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from arm_msgs.msg import MitState, MitCommand


def _duration_to_sec(d):
    return float(d.sec) + float(d.nanosec) * 1e-9


class MitBridge(Node):
    """
    통합 브릿지:
      - /arm/mit_state (MitState, 모터 1개씩) -> /joint_states (4개 묶어서)
      - /joint_trajectory (JointTrajectory) -> /arm/mit_cmd (MitCommand, 모터 1개씩 스트리밍)
    """

    def __init__(self):
        super().__init__("mit_bridge")

        # ===== 매핑(필요하면 여기만 수정) =====
        self.joint_order = ["j1", "j2", "j3", "j4"]
        self.joint_to_motor = {"j1": 1, "j2": 2, "j3": 3, "j4": 4}
        self.motor_to_joint = {v: k for k, v in self.joint_to_motor.items()}

        # ===== 게인(필요하면 튜닝) =====
        self.kp = 30.0
        self.kd = 2.0
        self.tau_ff = 0.0

        # ===== 상태 저장 =====
        self.last_q = {jn: None for jn in self.joint_order}
        self.last_qd = {jn: 0.0 for jn in self.joint_order}

        # ===== trajectory 저장 =====
        self.traj = None
        self.traj_start_time = None
        self.name_to_index = None

        # QoS:
        # - mit_state는 센서 스트림이라 BEST_EFFORT로 맞추는 게 정답 (너희가 겪은 RELIABILITY 문제 해결)
        # - mit_cmd는 컨트롤 명령: RELIABLE로 보내도 되고, 보수적으로 BEST_EFFORT로 해도 됨
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        traj_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # subs/pubs
        self.sub_state = self.create_subscription(MitState, "/arm/mit_state", self.on_state, qos_profile_sensor_data)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 50)

        self.sub_traj = self.create_subscription(JointTrajectory, "/joint_trajectory", self.on_traj, traj_qos)
        self.pub_cmd = self.create_publisher(MitCommand, "/arm/mit_cmd", qos_profile_sensor_data)
        # timers
        self.timer_js = self.create_timer(0.01, self.publish_joint_state)    # 100Hz
        self.timer_cmd = self.create_timer(0.005, self.publish_mit_cmd)      # 200Hz

        self.get_logger().info("MitBridge ready: /arm/mit_state->/joint_states and /joint_trajectory->/arm/mit_cmd")

    # ---------- mit_state -> joint_states ----------
    def on_state(self, msg: MitState):
        mid = int(msg.motor_id)
        if mid not in self.motor_to_joint:
            return
        jn = self.motor_to_joint[mid]
        self.last_q[jn] = float(msg.q)
        self.last_qd[jn] = float(msg.qd)

    def publish_joint_state(self):
        if any(self.last_q[jn] is None for jn in self.joint_order):
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_order
        js.position = [self.last_q[jn] for jn in self.joint_order]
        js.velocity = [self.last_qd[jn] for jn in self.joint_order]
        self.pub_js.publish(js)

    # ---------- joint_trajectory -> mit_cmd ----------
    def on_traj(self, msg: JointTrajectory):
        if not msg.joint_names or not msg.points:
            self.get_logger().warn("Received empty trajectory.")
            return

        self.traj = msg
        self.traj_start_time = self.get_clock().now()
        self.name_to_index = {n: i for i, n in enumerate(msg.joint_names)}
        self.get_logger().info(f"New trajectory received. joints={msg.joint_names} points={len(msg.points)}")

    def _sample_traj(self, t):
        """t초에서 (q, dq) 샘플. positions/velocities 선형보간."""
        pts = self.traj.points
        times = [_duration_to_sec(p.time_from_start) for p in pts]

        if t <= times[0]:
            p = pts[0]
            return p.positions, (p.velocities if p.velocities else None)

        if t >= times[-1]:
            p = pts[-1]
            return p.positions, (p.velocities if p.velocities else None)

        i = 1
        while i < len(times) and times[i] < t:
            i += 1

        p0, p1 = pts[i - 1], pts[i]
        t0, t1 = times[i - 1], times[i]
        alpha = (t - t0) / max(1e-9, (t1 - t0))

        q0, q1 = p0.positions, p1.positions
        q = [(1 - alpha) * a + alpha * b for a, b in zip(q0, q1)]

        if p0.velocities and p1.velocities:
            dq0, dq1 = p0.velocities, p1.velocities
            dq = [(1 - alpha) * a + alpha * b for a, b in zip(dq0, dq1)]
        else:
            dq = None

        return q, dq

    def publish_mit_cmd(self):
        if self.traj is None or self.traj_start_time is None or self.name_to_index is None:
            return

        t = (self.get_clock().now() - self.traj_start_time).nanoseconds * 1e-9
        q_all, dq_all = self._sample_traj(t)

        now = self.get_clock().now().to_msg()

        for jn in self.joint_order:
            if jn not in self.name_to_index:
                continue
            idx = self.name_to_index[jn]

            cmd = MitCommand()
            cmd.stamp = now
            cmd.motor_id = int(self.joint_to_motor[jn])
            cmd.q_des = float(q_all[idx])
            cmd.qd_des = float(dq_all[idx]) if dq_all is not None else 0.0
            cmd.kp = float(self.kp)
            cmd.kd = float(self.kd)
            cmd.tau_ff = float(self.tau_ff)

            self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = MitBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
