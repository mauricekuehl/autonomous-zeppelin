from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

SERVO_GPIO_PORT_LEFT = 1
SERVO_GPIO_PORT_RIGHT = 0
MIN_PULSE_WIDTH = 0.000540
MAX_PULSE_WIDTH = 0.002470
PIN_FACTORY = PiGPIOFactory()


class Drive(Node):
    def __init__(self):
        super().__init__("drive_hardware")
        self.power_right = 0
        self.power_left = 0

        self.x_linear = 0
        self.z_angular = 0

        self.last_time = time.time()

        self.MASS = 0.7  # kg
        self.ACCELERATION = 1  # m/s²
        self.ACCELERATION_ANGULAR = 1  # m/s² when Motors are spinning in oposite Direction TODO how to calc and does this even make sense + general Angular stuff
        self.AIR_FRICTION_LINEAR = 0.5

        self.servo_left = AngularServo(
            SERVO_GPIO_PORT_LEFT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
            initial_angle=0,
            min_angle=-1,
            max_angle=1,
        )
        self.servo_right = AngularServo(
            SERVO_GPIO_PORT_RIGHT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
            initial_angle=0,
            min_angle=-1,
            max_angle=1,
        )

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.odometry_publisher = self.create_publisher(Odometry, "motor/odom", 1)

    def cmd_vel_callback(self, twist):
        lx = twist.linear.x
        ly = twist.linear.y
        lz = twist.linear.z

        ax = twist.angular.x
        ay = twist.angular.y
        az = twist.angular.z

    def set_max_min_esc(self):
        self.move(1, 1)
        time.sleep(2)
        self.move(-1, -1)
        time.sleep(1)
        self.move(0, 0)

    def move(self, left, right):
        # TODO -1 != 1 is terms of Force
        self.servo_left.angle = left
        self.servo_right.angle = right

    def stop(self):
        self.servo_left.angle = 0
        self.servo_right.angle = 0

    def step(self, ms):
        super().step(ms)
        d_time = time.time() - self.last_time  # Seconds

        # Magic:
        acceleration_linear_x = (
            (self.power_right + self.power_left) * 0.5 * self.ACCELERATION
        )
        acceleration_angular_z = (
            (self.power_right - self.power_left) * 0.5 * self.ACCELERATION_ANGULAR
        )

        # TODO this but with Drag
        self.x_linear = self.x_linear + acceleration_linear_x * d_time

        self.last_time = d_time

        stamp = Time(seconds=self.robot.getTime()).to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 0
        msg.child_frame_id = 0
        msg.twist.twist.linear.x = 0
        msg.twist.twist.angular.z = 0
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 0
        self.odometry_publisher.publish(msg)


def main():
    rclpy.init()
    drive = Drive()
    rclpy.spin(drive)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
