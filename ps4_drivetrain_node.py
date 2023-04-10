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
import RPi.GPIO as GPIO
from rclpy.node import Node
from sensor_msgs.msg import Joy


class PS4DrivetrainNode(Node):
    def __init__(self):
        super().__init__('ps4_drivetrain_node')
        self.subscription = self.create_subscription(Joy, 'ps4_joy', self.joy_callback, 10)
        self.subscription

        # Set GPIO pin assignments for motors
        self.left_motor_pwm = 15
        self.left_motor_dir = 19
        self.right_motor_pwm = 11
        self.right_motor_dir = 13

        # Initialize RPi.GPIO and set GPIO pins as output
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_motor_pwm, GPIO.OUT)
        GPIO.setup(self.left_motor_dir, GPIO.OUT)
        GPIO.setup(self.right_motor_pwm, GPIO.OUT)
        GPIO.setup(self.right_motor_dir, GPIO.OUT)

        # Initialize PWM
        self.left_motor = GPIO.PWM(self.left_motor_pwm, 100)
        self.right_motor = GPIO.PWM(self.right_motor_pwm, 100)
        self.left_motor.start(0)
        self.right_motor.start(0)

    def joy_callback(self, msg):
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]

        left_speed = (left_stick_y + left_stick_x) * 100
        right_speed = (left_stick_y - left_stick_x) * 100

        self.set_motor_speed(self.left_motor, self.left_motor_dir, left_speed)
        self.set_motor_speed(self.right_motor, self.right_motor_dir, right_speed)

    def set_motor_speed(self, motor, dir_pin, speed):
        direction = 1 if speed >= 0 else 0
        speed = abs(speed)

        GPIO.output(dir_pin, direction)
        motor.ChangeDutyCycle(speed)

    def on_shutdown(self):
        self.left_motor.stop()
        self.right_motor.stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    ps4_drivetrain_node = PS4DrivetrainNode()

    rclpy.on_shutdown(ps4_drivetrain_node.on_shutdown)

    rclpy.spin(ps4_drivetrain_node)
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