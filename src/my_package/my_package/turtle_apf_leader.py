import rclpy 
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs import Twist
import math
import random

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

        #publisher+timer
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel/', 10)
        self.create_timer(0.05, self.control_loop)


        @staticmethod
        def random_goal(margin, float = 1.0) -> tuple[float, float]:
            return (
                random.uniform(margin, 11.0 - margin),
                random.uniform(margin, 11.0 - margin)
            )
        #APF core
        def attractive_force(self) -> tuple[float, float]:
            ka = self.get_parameters('ka').value
            return ka*(self.goal_x - self.pose.x), ka*(self.goal_y - self.pose.y)
        
        def repulsive_force(self) -> tuple[float, float]:
            kr = self.get_parameters('kr').value
            rho_0 = self.get_parameters('rho_0').value
            fx = 0
            fy = 0

            for obstacle in self.obstacle_poses.values():
                dx = self.pose.x - obstacle.x
                dy = self.pose.y - obstacle.y
                rho = math.hypot(dx, dy)
                if 0.0 < rho < rho_0:
                    coef = kr*(1.0/rho - 1.0/rho_0)/rho ** 2
                    fx += coef * dx/rho
                    fy += coef * dy/rho

            return fx, fy
        
        def leader_cb(self, msg: Pose):
            self.pose = msg

        def control_loop(self):
            if self.pose is None:
                return
            
            goal_radius = self.get_parameter('goal_radius').value()
            max_linear = self.get_parameter('max_linear').value()
            max_angular = self.get_parameter('max_angular').value()
            kp_angular = self.get_parameter('kp_angular').value()

            
        


        









