"""
TO CREATE THE NODE

1. install these packages

sudo apt update
sudo apt install ros-humble-joy
sudo apt install python3-pigpio

2. create a new ROS2 package, we change directory to src

ros2 pkg create ps4_drivetrain --build-type ament_python --dependencies rclpy joy

As gpio is not a ROS package, you can manually add the pigpio dependency to the setup.cfg file of your package after creating it.

[options]
setup_requires =
    setuptools>=54.1.0
install_requires =
    rclpy
    sensor_msgs
    pigpio

3. create the python file for the node

cd ps4_drivetrain
mkdir -p src/ps4_drivetrain
touch src/ps4_drivetrain/ps4_drivetrain_node.py
chmod +x src/ps4_drivetrain/ps4_drivetrain_node.py

4. open ps4_drivetrain_node.py to use the code below

"""

import rclpy
import pigpio
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4DrivetrainNode(Node):
    def __init__(self):
        super().__init__('ps4_drivetrain_node') #name for the node
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10) # creates ROS2 subscription to listen to 'Joy' messages published on the 'joy topic'
        self.subscription  # prevent unused variable warning

        # Set GPIO pin assignments for motors
        self.left_motor_pwm = 15
        self.left_motor_dir = 19
        self.right_motor_pwm = 11
        self.right_motor_dir = 13

        # Initialize pigpio and set GPIO pins as output
        self.pi = pigpio.pi()
        self.pi.set_mode(self.left_motor_pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.left_motor_dir, pigpio.OUTPUT)
        self.pi.set_mode(self.right_motor_pwm, pigpio.OUTPUT)
        self.pi.set_mode(self.right_motor_dir, pigpio.OUTPUT)

    def joy_callback(self, msg):
        left_stick_x = msg.axes[0]  # Left stick horizontal axis (left/right)
        left_stick_y = msg.axes[1]  # Left stick vertical axis (up/down)

        # Calculate motor speeds based on left stick input
        left_speed = (left_stick_y + left_stick_x) * 255  # Scale to [0, 255] for MDD20A for the speed of the motor
        right_speed = (left_stick_y - left_stick_x) * 255  # Scale to [0, 255] for MDD20A for the speed of the motor

        # Set motor speeds and directions
        self.set_motor_speed(self.left_motor_pwm, self.left_motor_dir, left_speed)
        self.set_motor_speed(self.right_motor_pwm, self.right_motor_dir, right_speed)

    def set_motor_speed(self, pwm_pin, dir_pin, speed):
        # Set motor speed and direction based on the given speed value
        if speed > 0:
            self.pi.write(dir_pin, 1)
        else:
            self.pi.write(dir_pin, 0)

        self.pi.set_PWM_dutycycle(pwm_pin, abs(speed))
        
    # def set_motor_speed(self, pwm_pin, dir_pin, speed):
    #     # Set motor speed and direction based on the given speed value
    #     direction = 1 if speed >= 0 else 0
    #     speed = abs(speed)

    #     self.pi.write(dir_pin, direction)
    #     self.pi.set_PWM_dutycycle(pwm_pin, speed)

    def on_shutdown(self):
        # Stop motors and release GPIO resources
        self.pi.set_PWM_dutycycle(self.left_motor_pwm, 0)
        self.pi.set_PWM_dutycycle(self.right_motor_pwm, 0)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    ps4_drivetrain_node = PS4DrivetrainNode()

    # Register custom shutdown callback
    rclpy.on_shutdown(ps4_drivetrain_node.on_shutdown)

    rclpy.spin(ps4_drivetrain_node) # infinite loop for running the code similar to "while(1){...}"
    ps4_drivetrain_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
Then open the setup.py and modify the entry point as below

entry_points={
    'console_scripts': [
        'ps4_drivetrain_node = ps4_drivetrain.ps4_drivetrain_node:main',
    ],
},

Then,we can build the node by runing these commands

cd ~/ros2_ws
colcon build
source install/setup.bash (or "source ~/.bashrc")

terminal 1: 
source install/setup.bash (or "source ~/.bashrc")
ros2 run ps4_drivetrain ps4_drivetrain_node

terminal 2: 
source install/setup.bash (or "source ~/.bashrc")
ros2 run joy joy_node
"""