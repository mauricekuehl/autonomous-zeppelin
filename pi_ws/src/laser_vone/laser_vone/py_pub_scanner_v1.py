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
DEGREE = 30

# Lidar
SERIAL_PATH = "/dev/ttyS0"
COM_RATE = 115200

servo = AngularServo(
    SERVO_GPIO_PORT,
    min_pulse_width=MIN_PULSE_WIDTH,
    max_pulse_width=MAX_PULSE_WIDTH,
    pin_factory=PIN_FACTORY,
)


class Scanner(Node):
    def __init__(self):
        super().__init__("py_pub_scanner_v1_node")
        self.ser = serial.Serial(SERIAL_PATH, COM_RATE)
        self.publisher = self.create_publisher(LaserScan, "scan", 1)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.servo = servo

    def run(self):
        if self.ser.is_open == False:
            self.ser.open()

        arr = []
        where_in_middle = False
        RPM = 2
        HZ = 1000
        MIN_LENGTH = 30

        self.servo.angle = 30

        err = 0
        good = 0
        skip = 0

        counter = 0
        while True:
            counter += 1
            if counter > 10:
                while GPIO.input(INPUT_PIN) == 0:
                    pass
                t = time.time()
                while GPIO.input(INPUT_PIN) == 1:
                    pass
                degree = round(100000 * (time.time() - t))
                counter = 0

                if degree >= 80 and degree <= 90:
                    where_in_middle = True
                elif (
                    where_in_middle
                    and degree >= 20
                    and degree <= 30
                    and len(arr) > MIN_LENGTH
                ):
                    # if len(arr) == 470:
                    self.publish_data(arr)
                    where_in_middle = False
                    arr = []
                    print("good", good, "err", err, "skip", skip)
                    good = 0
                    err = 0
                    skip = 0

            count = self.ser.in_waiting
            if count > 8:
                recv = self.ser.read(9)
                self.ser.reset_input_buffer()
                if recv[0] == 0x59 and recv[1] == 0x59:
                    distance = recv[2] + recv[3] * 256
                    # strength = recv[4] + recv[5] * 256
                    arr.append(float(distance / 100))
                    # print(distance / 100)
                    good += 1
                else:
                    # print("err")
                    err += 1
            else:
                skip += 1

    def publish_data(self, distance_arr):
        # print(distance_arr)
        taget_length = 170
        print(len(distance_arr) - taget_length)
        if len(distance_arr) < taget_length:
            return
        else:
            distance_arr = distance_arr[0:taget_length]

        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        # scan.header.frame_id = "map"
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.angle_min = 0.0
        scan.angle_max = math.radians(360)
        scan.angle_increment = float(math.radians(360) / len(distance_arr))
        print(
            scan.angle_increment * len(distance_arr) / math.radians(360),
            len(distance_arr),
        )
        scan.ranges = distance_arr

        self.publisher.publish(scan)


def main():
    rclpy.init()
    scanner = Scanner()
    try:
        scanner.run()
    except KeyboardInterrupt:
        servo.angle = 0
        print("ctrl+c")
        scanner.ser.close()
        # scanner.publisher.destroy_node()
        # rclpy.shutdown()
    finally:
        servo.angle = 0


if __name__ == "__main__":
    main()
