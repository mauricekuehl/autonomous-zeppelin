import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from gpiozero import AngularServo, LED
from gpiozero.pins.pigpio import PiGPIOFactory


# Servo
SERVO_GPIO_PORT = 17
MIN_PULSE_WIDTH = 0.000540
MAX_PULSE_WIDTH = 0.002470
PIN_FACTORY = PiGPIOFactory()

LED_GPIO_PORT = 27


class Controller_Lift(Node):
    def __init__(self):
        super().__init__("py_pub_scanner_v1_node")
        self.servo = AngularServo(
            SERVO_GPIO_PORT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
        )
        self.create_subscription(Twist, "cmd_vel_lift", self.cmd_vel_callback, 1)
        self.led = LED(LED_GPIO_PORT)
        self.last_linear_x = 0

    def cmd_vel_callback(self, msg):
        if self.last_linear_x != msg.linear.x:
            self.last_linear_x = msg.linear.x
            if msg.linear.x > 0:
                self.led.on()
                self.servo.angle = 90
            elif msg.linear.x < 0:
                self.led.on()
                self.servo.angle = -90
            else:
                self.led.off()


def main():
    rclpy.init()
    controller = Controller_Lift()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
