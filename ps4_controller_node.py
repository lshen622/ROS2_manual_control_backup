import rclpy
import pygame
from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import Joy


class PS4ControllerNode(Node):
    def __init__(self):
        # Initialize the ROS2 node with the name 'ps4_controller_node'
        super().__init__('ps4_controller_node')

        # Create a publisher for the 'ps4_joy' topic with message type Joy and a queue size of 10
        self.publisher = self.create_publisher(Joy, 'ps4_joy', 10)

        # Initialize the pygame library for accessing the PS4 controller
        pygame.init()
        pygame.joystick.init()

        # Get the first available joystick (the PS4 controller) and initialize it
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def process_controller_input(self):
        # Continuously process controller input while the ROS2 node is running
        while rclpy.ok():
            # Get all the events (inputs) from the controller
            for event in pygame.event.get():
                # Check if the event is of type JOYAXISMOTION (joystick axis movement)
                if event.type == pygame.JOYAXISMOTION:
                    # Get the x-axis and y-axis values of the left joystick
                    left_stick_x = self.controller.get_axis(0)
                    left_stick_y = self.controller.get_axis(1)

                    # Create a Joy message and populate the axes values
                    joy = Joy()
                    joy.axes = [left_stick_x, left_stick_y]

                    # Publish the Joy message containing the joystick axes values
                    self.publisher.publish(joy)

            # Sleep for a short time to reduce CPU usage
            sleep(0.1)


def main(args=None):
    # Initialize the ROS2 library
    rclpy.init(args=args)

    # Create an instance of the PS4ControllerNode class
    ps4_controller_node = PS4ControllerNode()

    # Start processing controller input and keep running until KeyboardInterrupt is raised (Ctrl+C)
    try:
        ps4_controller_node.process_controller_input()
    except KeyboardInterrupt:
        pass

    # Clean up the node resources before shutting down
    ps4_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
