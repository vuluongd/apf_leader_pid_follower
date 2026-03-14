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

        

                               

        



