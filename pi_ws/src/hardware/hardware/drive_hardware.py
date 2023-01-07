from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
from math import sin, cos
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SERVO_GPIO_PORT_LEFT = 23
SERVO_GPIO_PORT_RIGHT = 24
MIN_PULSE_WIDTH = 1000 / 1000000
MAX_PULSE_WIDTH = 2000 / 1000000
print("MIN_PULSE_WIDTH", MIN_PULSE_WIDTH)
print("MAX_PULSE_WIDTH", MAX_PULSE_WIDTH)
PIN_FACTORY = PiGPIOFactory()
# 0.07 dead band


class Controller(Node):
    def __init__(self):
        super().__init__("drive_hardware")
        self.power_right = 0  # -1 to 1
        self.power_left = 0  # -1 to 1

        self.x_linear = 0
        self.z_angular = 0

        self.last_time = time()

        """to messure without the robot"""
        self.MASS = 0.7  # kg
        self.MAX_POWER = 1  # 1 = 100% power
        self.ACCELERATION = 2  # m/s² for max power
        self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED = 1
        self.MIN_POWER = (
            0.001  # only provides a warning right know as nav2 should not undergo this
        )

        """to messure with helium for cmd_vel"""
        speed = 1  # m/s for max power
        power = self.MAX_POWER  # max power should be used for this, 50% maybe too
        self.POWER_FOR_ONE_METER_PER_SECOND = power / speed

        # tune angular movement, it does not really mean what the name says
        self.WHEEL_SEPERATION = 1  # m

        """to tune the odometry estimation"""
        self.ACCELERATION_ANGULAR = self.ACCELERATION  # m/s²

        # F_n = F_f = m * a = drag * v²
        # drag = m * a / v²
        self.AIR_FRICTION_LINEAR = self.MASS * self.ACCELERATION / speed**2
        self.AIR_FRICTION_ANGULAR = self.AIR_FRICTION_LINEAR * 5  # just a guess

        self.COVARIANCE_LINEAR_X = 0  # TODO
        self.COVARIANCE_ANGULAR_Z = 0

        self.COVARIANCE_MATRIX = [0 for i in range(36)]
        self.COVARIANCE_MATRIX[0] = self.COVARIANCE_LINEAR_X
        self.COVARIANCE_MATRIX[35] = self.COVARIANCE_ANGULAR_Z

        self.servo_left = AngularServo(
            SERVO_GPIO_PORT_LEFT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
            initial_angle=0,
            # min_angle=-1,
            # max_angle=1,
        )
        self.servo_right = AngularServo(
            SERVO_GPIO_PORT_RIGHT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
            initial_angle=0,
            # min_angle=-1,
            # max_angle=1,
        )

        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.odometry_publisher = self.create_publisher(Odometry, "motor/odom", 1)

        self.create_timer(0.02, self.step)

    def cmd_vel_callback(self, twist):
        power_right = (
            twist.linear.x + self.WHEEL_SEPERATION * twist.angular.z
        )  # this was for wheels...
        power_left = twist.linear.x - self.WHEEL_SEPERATION * twist.angular.z

        # TODO use polynomial function to map from twist to power
        self.power_left = power_left * self.POWER_FOR_ONE_METER_PER_SECOND
        self.power_right = power_right * self.POWER_FOR_ONE_METER_PER_SECOND
        self.execute_power_levels()

    def set_max_min_esc(self):
        self.set_power(1, 1)
        sleep(3)
        self.set_power(0, 0)

    def execute_power_levels(self):  # -1 = 1 in terms of thrust
        # this is just a warning with no huge consequences if it happens
        if (0 < self.power_left and self.power_left < self.MIN_POWER) or (
            0 < self.power_right and self.power_right < self.MIN_POWER
        ):
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

        if self.power_left < 0:
            power_left = self.power_left * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
        else:
            power_left = self.power_left
        if self.power_right < 0:
            power_right = self.power_right * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
        else:
            power_right = self.power_right

        self.set_power(power_left, power_right)

    def set_power(self, power_left, power_right):
        print("executing power levels", power_left, power_right, "")
        self.servo_left.angle = power_left * 90
        self.servo_right.angle = power_right * 90

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
        msg.twist.twist.linear.x = self.x_linear  # m/s
        # msg.twist.covariance = self.COVARIANCE_MATRIX
        msg.twist.twist.angular.z = self.z_angular  # radians/second
        self.odometry_publisher.publish(msg)


def main():
    rclpy.init()
    controller = Controller()
    i = input("Press s to set max and min esc values")
    if i == "s":
        controller.set_max_min_esc()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
