import os
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .robot_model import RobotModel
from .ik import solve_ik_position_only
from .traj import make_quintic_joint_traj
pkg_share = get_package_share_directory("arm_sim")
urdf_path = os.path.join(pkg_share, "urdf", "robot.urdf")

ee_frame = "ee_link"
base_frame = "base"

class IKPlannerNode(Node):
    def __init__(self):
        super().__init__("ik_planner_node")

        # ====== parameters ======
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("ee_frame", "ee_link")
        self.declare_parameter("base_frame", "base")     # goal frame_id 체크용
        self.declare_parameter("traj_T", 3.0)
        self.declare_parameter("traj_dt", 0.01)

        # cost weights
        self.declare_parameter("w_pos", 1.0)
        self.declare_parameter("w_move", 0.08)
        self.declare_parameter("w_lim", 0.03)
        self.declare_parameter("n_restarts", 10)


        if not urdf_path or not os.path.exists(urdf_path):
            raise RuntimeError("Parameter 'urdf_path' is empty or file does not exist.")

        self.robot = RobotModel(urdf_path, ee_frame=ee_frame)

        self.joint_names = None
        self.q_now = None

        self.sub_js = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.sub_goal = self.create_subscription(PointStamped, "/goal_point", self._on_goal, 10)

        self.pub_traj = self.create_publisher(JointTrajectory, "/joint_trajectory", 10)

        self.get_logger().info(f"IKPlanner ready. ee_frame={ee_frame}, urdf={urdf_path}")

    def _on_joint_state(self, msg: JointState):
        if self.joint_names is None:
            # 첫 joint_states로 순서를 고정
            self.joint_names = list(msg.name)
            self.get_logger().info(f"Joint order locked: {self.joint_names}")

        # joint_names 기준으로 q_now 정렬
        name_to_idx = {n:i for i,n in enumerate(msg.name)}
        q = []
        for n in self.joint_names:
            if n not in name_to_idx:
                return
            q.append(msg.position[name_to_idx[n]])
        self.q_now = np.array(q, dtype=float)

    def _on_goal(self, msg: PointStamped):
        if self.q_now is None or self.joint_names is None:
            self.get_logger().warn("No joint_states yet.")
            return

        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        if msg.header.frame_id and msg.header.frame_id != base_frame:
            self.get_logger().warn(f"Goal frame_id='{msg.header.frame_id}' != '{base_frame}'. (Assuming already in base frame)")

        p_des = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

        w_pos = self.get_parameter("w_pos").value
        w_move = self.get_parameter("w_move").value
        w_lim = self.get_parameter("w_lim").value
        n_restarts = int(self.get_parameter("n_restarts").value)

        try:
            q_star, info = solve_ik_position_only(
                self.robot, p_des, self.q_now,
                w_pos=w_pos, w_move=w_move, w_lim=w_lim,
                n_restarts=n_restarts
            )
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        T = float(self.get_parameter("traj_T").value)
        dt = float(self.get_parameter("traj_dt").value)
        ts, qs, dqs = make_quintic_joint_traj(self.q_now, q_star, T=T, dt=dt)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # JointTrajectoryPoint 채우기
        for t, q, dq in zip(ts, qs, dqs):
            p = JointTrajectoryPoint()
            p.positions = q.tolist()
            p.velocities = dq.tolist()
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(p)

        self.pub_traj.publish(traj)
        self.get_logger().info(f"Planned to p={p_des.tolist()} | cost={info['cost']:.4e} nfev={info['nfev']}")
        

def main():
    rclpy.init()
    node = IKPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
