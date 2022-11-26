# from gpiozero import AngularServo
# from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SERVO_GPIO_PORT_LEFT = 1
SERVO_GPIO_PORT_RIGHT = 0
MIN_PULSE_WIDTH = 0.000540
MAX_PULSE_WIDTH = 0.002470
# PIN_FACTORY = PiGPIOFactory()


class Controller(Node):
    def __init__(self):
        super().__init__("drive_hardware")
        self.power_right = 0
        self.power_left = 0

        self.x_linear = 0
        self.z_angular = 0
        self.x_pos = 0
        self.y_pos = 0
        self.angle = 0

        self.last_time = time()

        self.MASS = 0.7  # kg
        self.ACCELERATION = 0.2  # m/s²
        self.ACCELERATION_ANGULAR = 0.2  # m/s² when Motors are spinning in oposite Direction TODO: how to calc and does this even make sense + general Angular stuff
        self.AIR_FRICTION_LINEAR = 20
        self.AIR_FRICTION_ANGULAR = 100
        self.RADIUS_MOTORS = 0.5  # m

        # self.servo_left = AngularServo(
        #     SERVO_GPIO_PORT_LEFT,
        #     min_pulse_width=MIN_PULSE_WIDTH,
        #     max_pulse_width=MAX_PULSE_WIDTH,
        #     pin_factory=PIN_FACTORY,
        #     initial_angle=0,
        #     min_angle=-1,
        #     max_angle=1,
        # )
        # self.servo_right = AngularServo(
        #     SERVO_GPIO_PORT_RIGHT,
        #     min_pulse_width=MIN_PULSE_WIDTH,
        #     max_pulse_width=MAX_PULSE_WIDTH,
        #     pin_factory=PIN_FACTORY,
        #     initial_angle=0,
        #     min_angle=-1,
        #     max_angle=1,
        # )

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.odometry_publisher = self.create_publisher(Odometry, "motor/odom", 1)

        self.create_timer(0.02, self.step)

    def cmd_vel_callback(self, twist):
        # TODO set Motor speed with inclusion of the thrust in the opposite direction
        right_velocity = (
            twist.linear.x + self.RADIUS_MOTORS * twist.angular.z
        )  # this was for wheels...
        left_velocity = twist.linear.x - self.RADIUS_MOTORS * twist.angular.z
        self.move(left_velocity, right_velocity)

    def set_max_min_esc(self):
        self.move(1, 1)
        time.sleep(2)
        self.move(-1, -1)
        time.sleep(1)
        self.move(0, 0)

    def move(self, left, right):
        # TODO -1 != 1 in terms of Force
        # self.servo_left.angle = left
        # self.servo_right.angle = right
        self.power_left = left
        self.power_right = right

    def step(self):
        time_now = time()
        d_time = time_now - self.last_time  # Seconds
        self.last_time = time_now

        """ Magic: """
        acceleration_linear_x = (
            (self.power_right + self.power_left) * 0.5 * self.ACCELERATION
        )
        acceleration_angular_z = (
            (self.power_right - self.power_left) * 0.5 * self.ACCELERATION_ANGULAR
        )

        net_force_linear_x = (
            acceleration_linear_x * self.MASS
            - self.AIR_FRICTION_LINEAR * self.x_linear * abs(self.x_linear)
        )
        net_force_angular_z = (
            acceleration_angular_z * self.MASS
            - self.AIR_FRICTION_ANGULAR * self.z_angular * abs(self.z_angular)
        )

        self.x_linear += net_force_linear_x / self.MASS * d_time
        self.z_angular += (
            net_force_angular_z / self.MASS * d_time
        )  # TODO this is stupid right?

        self.angle += self.z_angular * d_time

        self.x_pos += (
            self.x_linear * d_time * cos(self.angle)
        )  # maybe use this: s = s0 + v0 · t + 1/2 · a · t²
        self.y_pos += self.x_linear * d_time * sin(self.angle)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = self.x_linear
        msg.twist.twist.angular.z = self.z_angular  # radians/second
        msg.pose.pose.position.x = self.x_pos
        msg.pose.pose.position.y = self.y_pos
        msg.pose.pose.orientation.z = sin(self.angle / 2)
        msg.pose.pose.orientation.w = cos(self.angle / 2)
        self.odometry_publisher.publish(msg)


def main():
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
