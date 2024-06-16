import rclpy
import keyboard
from .navigation_controller import NavigationController


def main(args=None):
    rclpy.init(args=args)
    navigation_controller = NavigationController()

    print("Press '1' to enable AGV, 'ESC' to disable AGV.")
    print("Use 'W', 'A', 'S', 'D' keys to move the AGV.")
    print("Press 'Q' to stop the AGV.")

    try:
        while True:

            # Enable / Disable
            if keyboard.is_pressed('1'):
                navigation_controller.enable_agv()
            elif keyboard.is_pressed('esc'):
                navigation_controller.disable_agv()

            # move
            elif keyboard.is_pressed('w'):
                navigation_controller.move_agv(0.5, 0.0, 0.0)  # Move forward
            elif keyboard.is_pressed('s'):
                navigation_controller.move_agv(-0.5, 0.0, 0.0)  # Move backward
                
            # rotate
            elif keyboard.is_pressed('a'):
                navigation_controller.move_agv(0.0, 0.0, 0.5)  # Rotate left
            elif keyboard.is_pressed('d'):
                navigation_controller.move_agv(0.0, 0.0, -0.5)  # Rotate right

            # stop
            elif keyboard.is_pressed('q'):
                navigation_controller.move_agv(0.0, 0.0, 0.0)  # Stop movement
    except KeyboardInterrupt:
        pass

    navigation_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

