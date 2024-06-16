import rclpy
from rclpy.node import Node
from interface.srv import AgvCommd

# for keyboard input
import curses

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
  

def main(stdscr):
    rclpy.init(args=None)
    navigation_controller = NavigationController()

    print("Press '1' to enable AGV, 'ESC' to disable AGV.")
    print("Use 'W', 'A', 'S', 'D' keys to move the AGV.")
    print("Press 'Q' to stop the AGV.")

    stdscr.nodelay(1)
    stdscr.timeout(100)

    while True:
        key = stdscr.getch()
        if key == ord('1'):
            navigation_controller.enable_agv()
        elif key == 27:  # ESC key
            navigation_controller.disable_agv()
            break
        elif key == ord('2'):
            navigation_controller.move_agv(0.0, 0.0, 0.0)  # Stop movement
        elif key == ord('w'):
            navigation_controller.move_agv(0.5, 0.0, 0.0)  # Move forward
        elif key == ord('s'):
            navigation_controller.move_agv(-0.5, 0.0, 0.0)  # Move backward
        elif key == ord('a'):
            navigation_controller.move_agv(0.0, 0.0, 0.5)  # Rotate left
        elif key == ord('d'):
            navigation_controller.move_agv(0.0, 0.0, -0.5)  # Rotate right
        elif key == ord('q'):
            navigation_controller.move_agv(0.0, 0.5, 0.0)  # Move left
        elif key == ord('e'):
            navigation_controller.move_agv(0.0, -0.5, 0.0)  # Move right

    navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)

