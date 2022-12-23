import rclpy
from rclpy.node import Node
import serial
import time
import math
import RPi.GPIO as GPIO
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from sensor_msgs.msg import LaserScan

# Servo
INPUT_PIN = 18
SERVO_GPIO_PORT = 17
MIN_PULSE_WIDTH = 1280 / 1000000
MAX_PULSE_WIDTH = 1720 / 1000000
PIN_FACTORY = PiGPIOFactory()
MIN_SPEED_FOR_ROTATION = 0.6
DEGREE = 20

# Lidar
SERIAL_PATH = "/dev/ttyS0"
COM_RATE = 115200


class Scanner(Node):
    def __init__(self):
        super().__init__("py_pub_scanner_v1_node")
        self.servo = AngularServo(
            SERVO_GPIO_PORT,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH,
            pin_factory=PIN_FACTORY,
        )
        self.ser = serial.Serial(SERIAL_PATH, COM_RATE)
        self.publisher = self.create_publisher(LaserScan, "scan", 1)
        self.DEGREE = DEGREE

    def run(self):
        if self.ser.is_open == False:
            self.ser.open()

        arr = []
        where_in_middle = False
        RPM = 2
        HZ = 1000
        MIN_LENGTH = HZ / RPM * 0.9

        while True:
            degree = self.get_degree()
            if degree >= 80 and degree <= 90:
                where_in_middle = True
            elif (
                where_in_middle
                and degree >= 20
                and degree <= 30
                and len(arr) > MIN_LENGTH
            ):
                self.publish_data(arr)
                where_in_middle = False
                arr = []
            arr.append(self.get_distance())

    def get_distance(self):
        count = self.ser.in_waiting
        if count > 8:
            recv = self.ser.read(9)
            self.ser.reset_input_buffer()
            if recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + recv[3] * 256
                # strength = recv[4] + recv[5] * 256
                return distance
            else:
                print("bad 1")
                return 0
        else:
            print("bad 2")
            return 0

    def get_degree(self):
        while GPIO.input(INPUT_PIN) == 0:
            pass
        t = time.time()
        while GPIO.input(INPUT_PIN) == 1:
            pass
        print(round(100000 * (time.time() - t)))
        return round(100000 * (time.time() - t))

    def publish_data(self, distance_arr):
        # print(distance_arr)

        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        # scan.header.frame_id = "map"
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.angle_min = math.radians(-180)
        scan.angle_max = math.radians(180)
        #        scan.angle_increment = float(math.radians(180.2) / len(distance_arr))
        # scan.angle_increment = float(math.radians(180.4) / len(distance_arr))
        # scan.time_increment = float(self.SLEEP)
        scan.time_increment = 0.0
        scan.ranges = distance_arr

        self.publisher.publish(scan)


def main():
    rclpy.init()
    scanner = Scanner()
    try:
        scanner.run()
    except KeyboardInterrupt:
        print("ctrl+c")
        scanner.ser.close()
        # scanner.publisher.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
