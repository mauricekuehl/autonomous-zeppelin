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

# with setup
SETUP = True

"""
Notes:
The Max Speed for Nav2 is the speed for max power
Max Power should be set low by default

TODO max power backwards
TODO radian speed
TODO calculate air resistance into cmd_vel subscriber
TODO kill switch
"""


class Controller(Node):
    def __init__(self):
        super().__init__("drive_hardware")
        self.count = 0
        self.power_right = 0  # -1 to 1
        self.power_left = 0  # -1 to 1

        self.x_linear = 0
        self.z_angular = 0

        self.last_time = time()

        """to messure without the robot"""
        self.MASS = 0.7  # kg
        self.MAX_POWER = 1  # 1 = 100% power

        # at max power
        self.FORCE_FORWARD_NEWTON = 3
        self.FORCE_BACKWARDS_NEWTON = 1

        self.MIN_POWER = 0.05  # only provides a warning right know as nav2 should not undergo this and if so it is set to min power
        # TDDO: uns polynomial if power messurement is shit

        """caclulation"""
        self.FORCE = self.FORCE_FORWARD_NEWTON

        self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED = (
            self.FORCE_FORWARD_NEWTON / self.FORCE_BACKWARDS_NEWTON
        )

        """to messure with helium for cmd_vel"""
        speed = 1  # m/s for max power
        self.POWER_FOR_ONE_METER_PER_SECOND = self.MAX_POWER / speed

        # tune angular movement, it does not really mean what the name says. Higher = faster turns
        self.WHEEL_SEPERATION = 1  # m

        """to tune the odometry estimation"""
        self.FORCE_ANGULAR = self.FORCE  # m/s²

        # F_n = F_f = m * a = drag * v²
        # drag = m * a / v²
        self.AIR_FRICTION_LINEAR = self.FORCE / speed**2
        self.AIR_FRICTION_ANGULAR = self.AIR_FRICTION_LINEAR * 3  # just a guess

        self.COVARIANCE_MATRIX = [0.0 for i in range(36)]
        self.COVARIANCE_MATRIX[0] = 0.1
        # set covariance for linear y
        self.COVARIANCE_MATRIX[7] = 0.1
        # set covariance for linear z
        self.COVARIANCE_MATRIX[14] = 0.0
        # set covariance for angular x
        self.COVARIANCE_MATRIX[21] = 0.0
        # set covariance for angular y
        self.COVARIANCE_MATRIX[28] = 0.0
        # set covariance for angular z
        self.COVARIANCE_MATRIX[35] = 0.1

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

        self.create_timer(0.02, self.step)

    def cmd_vel_callback(self, twist):
        print("into---", twist.linear.x, twist.angular.z)
        power_right = (
            twist.linear.x + self.WHEEL_SEPERATION * twist.angular.z
        )  # this was for wheels...
        power_left = twist.linear.x - self.WHEEL_SEPERATION * twist.angular.z

        print("in", power_right, power_left)

        # TODO use polynomial function to map from twist to power
        self.power_left = power_left * self.POWER_FOR_ONE_METER_PER_SECOND
        self.power_right = power_right * self.POWER_FOR_ONE_METER_PER_SECOND
        self.execute_power_levels()

    def set_max_min_esc(self):
        self.servo_left.angle = 1
        self.servo_right.angle = 1

        input("press enter to set min")
        self.servo_left.angle = 0
        self.servo_right.angle = 0

    def get_pos_neg(self, power):
        if power > 0:
            return 1
        else:
            return -1

    def get_force_for_speed_x(self, speed):
        # F_f = F_n = drag * v²
        return self.AIR_FRICTION_LINEAR * speed**2 * self.get_pos_neg(speed)

    def get_force_for_speed_z(self, speed):
        # F_f = F_n = drag * v²
        return self.AIR_FRICTION_LINEAR * speed**2 * self.get_pos_neg(speed)

    def execute_power_levels(self):  # -1 = 1 in terms of thrust
        # this is just a warning with no huge consequences if it happens
        if (0 < abs(self.power_left) and abs(self.power_left) < self.MIN_POWER) or (
            0 < abs(self.power_right) and abs(self.power_right) < self.MIN_POWER
        ):
            print(
                "WARNING: Power too low",
                "l",
                self.power_left,
                "r",
                self.power_right,
            )
            if abs(self.power_left) < self.MIN_POWER:
                self.power_left = self.MIN_POWER * self.get_pos_neg(self.power_left)
            elif abs(self.power_right) < self.MIN_POWER:
                self.power_right = self.MIN_POWER * self.get_pos_neg(self.power_right)

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
                self.power_left = self.MAX_POWER * self.get_pos_neg(self.power_left)
                self.power_right = self.MAX_POWER * self.get_pos_neg(self.power_right)
            elif abs(self.power_left) > self.MAX_POWER:
                self.power_right = self.power_right / abs(self.power_left)
                self.power_left = self.MAX_POWER * self.get_pos_neg(self.power_left)
            elif abs(self.power_right) > self.MAX_POWER:
                self.power_left = self.power_left / abs(self.power_right)
                self.power_right = self.MAX_POWER * self.get_pos_neg(self.power_right)

        if self.power_left < 0:
            self.power_left = 0
            power_left = self.power_left * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
        else:
            power_left = self.power_left
        if self.power_right < 0:
            self.power_right = 0
            power_right = self.power_right * self.HOW_MUCH_MORE_POWER_BACKWARDS_NEEDED
        else:
            power_right = self.power_right

        print("-", self.power_left, self.power_right)

        # self.servo_left.angle = power_left
        # self.servo_right.angle = power_right

    def step(self):
        self.count += 1
        if self.count % 100 == 0:
            print("step")
            self.count = 0
        time_now = time()
        d_time = time_now - self.last_time  # Seconds
        self.last_time = time_now

        """ Magic: """
        force_linear_x = (self.power_right + self.power_left) * 0.5 * self.FORCE
        force_angular_z = (
            (self.power_right - self.power_left) * 0.5 * self.FORCE_ANGULAR
        )

        net_force_linear_x = (
            force_linear_x
            - self.AIR_FRICTION_LINEAR * self.x_linear * abs(self.x_linear)
        )
        net_force_angular_z = (
            force_angular_z
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
        msg.twist.covariance = self.COVARIANCE_MATRIX
        msg.twist.twist.angular.z = self.z_angular  # radians/second
        self.odometry_publisher.publish(msg)


def main():
    rclpy.init()
    controller = Controller()
    if SETUP:
        i = input("Press s to set max and min esc values")
        if i == "s":
            controller.set_max_min_esc()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
