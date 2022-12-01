# from gpiozero import AngularServo
# from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
from math import sin, cos
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

        # messure without the zeppelin
        self.MASS = 0.7  # kg
        self.ACCELERATION = 0.2  # m/s²
        self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED = 1.5
        self.MAX_POWER = 1  # 1 = 100% power
        self.MIN_POWER = (
            0.05  # only provides a warning right know as nav2 should not undergo this
        )

        # edge case, it does not really mean what the name says, TODO: effect in combination with ACCELERATION_ANGULAR
        self.WHEEL_SEPERATION = 1  # m
        # corelates with WHEEL_SEPERATION
        self.ACCELERATION_ANGULAR = 0.2  # m/s² TODO: how to calc and does this even make sense + general Angular stuff

        # messure first, first get the Zepplin to move like it should (cmd_vel)
        self.POWER_FOR_ONE_METER_PER_SECOND = 0.5

        # messure next, TODO maybe with the POWER_FOR_ONE_METER_PER_SECOND calculateble
        self.AIR_FRICTION_LINEAR = 20
        self.AIR_FRICTION_ANGULAR = 100

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
        right_power = (
            twist.linear.x + self.WHEEL_SEPERATION * twist.angular.z
        )  # this was for wheels...
        left_power = twist.linear.x - self.WHEEL_SEPERATION * twist.angular.z

        # TODO use polynomial function to map from twist to power
        self.left_power = left_power * self.POWER_FOR_ONE_METER_PER_SECOND
        self.right_power = right_power * self.POWER_FOR_ONE_METER_PER_SECOND
        self.move()

    def set_max_min_esc(self):
        pass

    def move(self):  # -1 = 1 in terms of thrust
        if self.power_left < 0:
            self.power_left = (
                self.power_left * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
            )
        if self.power_right < 0:
            self.power_right = (
                self.power_right * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
            )

        # this is just a warning with no huge consequences if is happens
        if self.power_left < self.MIN_POWER or self.power_right < self.MIN_POWER:
            print(
                "WARNING: Power too low",
                "l",
                self.power_left,
                "r",
                self.power_right,
            )
        # this should not happen, but just in case:
        if (
            abs(self.power_left) > self.MAX_POWER
            or abs(self.power_right) > self.MAX_POWER
        ):
            print(
                "ERROR: Motor power too high",
                "l",
                self.power_left,
                "r",
                self.power_right,
            )
            if (
                abs(self.power_left) > self.MAX_POWER
                and abs(self.power_right) > self.MAX_POWER
            ):
                self.power_left = self.MAX_POWER
                self.power_right = self.MAX_POWER
            elif abs(self.power_left) > self.MAX_POWER:
                self.power_right = self.power_right / self.power_left
                self.power_left = self.MAX_POWER
            elif abs(self.power_right) > self.MAX_POWER:
                self.power_left = self.power_left / self.power_right
                self.power_right = self.MAX_POWER

        self.servo_left.angle = self.power_left
        self.servo_right.angle = self.power_right

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
