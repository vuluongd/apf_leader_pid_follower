import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs import Twist
import math

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class PIDfollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        #pid cho follower
        self.declare_parameter('desired_distance',      1.5)
        self.declare_parameter('max_linear_speed',      2.0)
        self.declare_parameter('kp_linear',             1.5)
        self.declare_parameter('ki_linear',             0.04)
        self.declare_parameter('kd_linear',             0.2)
        self.declare_parameter('kp_angular',            1.5)
        self.declare_parameter('ki_angular',            0.04)
        self.declare_parameter('kd_angular',            0.4)
        self.declare_parameter('angle_threshold',       0.3)

        #trường đẩy tránh vật cản cho follower
        self.declare_parameter('kr_follower',           1.3)
        self.declare_parameter('rho_0',                 1.5)

        self.declare_parameter('obstacle_names', ['obstacle1', 'obstacle2', 'obstacle3'])

        self.leader_pose: Pose | None = None
        self.obstacle_poses: dict[str, Pose] = {}
        
        self.prev_distance_error = 0.0
        self.integral_distance = 0.0
        self.prev_angle_error = 0.0
        self.integral_angle = 0.0
        self.prev_time = self.get_clock().now()

        #subscriber cho leader và follower
        self.create_subscription(Pose, '/turtle1/pose', self.leader_cb, 10)
        self.create_subscription(Pose, '/turtle2.pose', self.follower_cb, 10)

        obstacle_names: list[str] = self.get_parameter('obstacle_names').value
        for name in obstacle_names:
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n = name : self.obstacle_poses.__setitem__(n, msg),
                10,
            )
        self.pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

    def leader_cb(self, msg: Pose):
        self.leader_pose = msg

    def follower_cb(self, msg: Pose):
        current_time = self.get_clock().now()
        dt = (current_time - prev_time) * 1e-9
        prev_time = current_time

        if dt <= 0 or self.leader_pose is None:
            return

        self.compute_and_publish(msg, dt)

    def repulsive_velocity(self, pose: Pose) -> tuple[float, float]:
        kr = self.get_parameter('kr_follower').value
        rho0 = self.get_parameter('rho_follower').value
        vx = vy = 0.0
        for obs in self.obstacle_poses.values():
            dx = pose.x - obs.x
            dy = pose.y - obs.y
            rho = math.hypot(dx, dy)

            if 0.0 < rho < rho0:

                coef = kr*(1.0 / rho - 1 /rho0)/rho**2
                vx += coef*dx/rho
                vy += coef*dy/rho

        return vx, vy
    
    #PID+APF

    def compute_and_publish(self, msg: Pose, dt: float):
        desired_distance = self.get_parameter('desired_distance').value
        max_linear_speed = self.get_parameter('max_linear_speed').value
        kp_linear = self.get_parameter('kp_linear').value
        ki_linear = self.get_parameter('ki_linear').value
        kd_linear = self.get_parameter('kd_linear').value
        kp_angular = self.get_parameter('kp_angular').value
        ki_angular = self.get_parameter('ki_angular').value
        kd_angular = self.get_parameter('kd_angular').value


                               

        



