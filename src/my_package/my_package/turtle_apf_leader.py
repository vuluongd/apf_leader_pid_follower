import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs import Twist
import math

class APFLeader(Node):
    def __init__(self):
        super().__init__('turtle_leader')

        self.declare_parameter('ka',            1.0)
        self.declare_parameter('kr',            2.0)
        self.declare_parameter('rho_0',          2.0)
        self.declare_parameter('goal_radius',   0.4)
        self.declare_parameter('max_linear',    2.0)
        self.declare_parameter('max_angular',    3.0)
        self.declare_parameter('kp_angular',    4.0)    

        self.declare_parameter('obstacle_names', ['obstacle1', 'obstacle2', 'obstacle3'])

        self.pose = Pose | None = None
        self.obstacle_poses: dict[str, Pose] = {}

        self.goal_x, self.goal_y = self.random_goal()

        self.get_logger().info(
            f'[APFleader] Initial goal -> ({self.goal_x:.2f}, self.goal_y:.2f)'
        )

        self.create_subscription(Pose, '/turtle1/pose', self.leader_cb, 10)
        
        obstacle_names: list[str] = self.get_parameter('obstacle_names').value

        for name in obstacle_names:
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n=name: self.obstacle_poses.__setitem__(n, msg),
                10,
            ) 

        







