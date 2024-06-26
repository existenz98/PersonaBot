import rclpy
from rclpy.node import Node
from interface.srv import AgvCommd

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


# enabled = 1: Enable the AGV,   -1: Disable the AGV,  0: Apply linear and rotation velocity

class NavigationController(Node):

    def __init__(self):
        super().__init__('navigation_controller')
        self.cli = self.create_client(AgvCommd, '/agv_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AgvCommd.Request()

    def send_request(self, enable, vx, vy, rot):
        self.req.enable = enable
        self.req.vx = vx
        self.req.vy = vy
        self.req.rot = rot

        # block, wait for completion
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def enable_agv(self):
        self.get_logger().info('Starting AGV...')
        response = self.send_request(1, 0.0, 0.0, 0.0)
        if response:
            self.get_logger().info(f'Success enable: {response.successenable}, Success move: {response.successmove}')
        else:
            self.get_logger().error('Failed to start AGV')

    def disable_agv(self):
        self.get_logger().info('Stopping AGV...')
        response = self.send_request(-1, 0.0, 0.0, 0.0)
        if response:
            self.get_logger().info(f'Success enable: {response.successenable}, Success move: {response.successmove}')
        else:
            self.get_logger().error('Failed to stop AGV')

    def move_agv(self, vx, vy, rot):
        self.get_logger().info(f'Moving AGV: vx={vx}, vy={vy}, rot={rot}')
        response = self.send_request(0, vx, vy, rot)
        if response:
            self.get_logger().info(f'Success enable: {response.successenable}, Success move: {response.successmove}')
        else:
            self.get_logger().error('Failed to move AGV')

