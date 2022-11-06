import rclpy
from rclpy.node import Node
import serial
import time
import math
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from sensor_msgs.msg import LaserScan

# Servo
SERVO_GPIO_PORT = 17
MIN_PULSE_WIDTH = 0.000540
MAX_PULSE_WIDTH = 0.002470
PIN_FACTORY = PiGPIOFactory()
MIN_SPEED_FOR_ROTATION = 0.6
DEGREE = 90

# Lidar
SERIAL_PATH = "/dev/ttyS0"
COM_RATE = 115200

# Speed, Steps
STEPS = 0.4  # 1 -> jedes grad
SPEED = 1  # max: 1
SUM_STEPS = DEGREE * 2 / STEPS
SLEEP = MIN_SPEED_FOR_ROTATION / SUM_STEPS / SPEED


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
        self.STEPS = STEPS
        self.SLEEP = SLEEP
        self.publisher = self.create_publisher(LaserScan, "scan", 1)
        self.DEGREE = DEGREE

    def run(self):
        print("run")

        if self.ser.is_open == False:
            self.ser.open()

        distance_arr = []
        distance = 0
        angle = 0
        count = 1
        good_data = 1
        err_count = 1
        sum_good = 1
        sum_bad = 1
        while True:
            time.sleep(self.SLEEP)
            if angle == self.DEGREE:
                sum_good += good_data
                sum_bad += err_count
                print(
                    "bad:",
                    err_count,
                    "good:",
                    good_data,
                    "ratio:",
                    good_data / err_count,
                )
                print(
                    "sum good:",
                    sum_good,
                    "sum bad:",
                    sum_bad,
                    "ratio:",
                    sum_good / sum_bad,
                )
                good_data = 1
                err_count = 1
                self.publish_data(distance_arr)  # invert on way back
                distance_arr = []
                angle = self.DEGREE * -1
                self.servo.angle = angle
                time.sleep(0.65)
            angle = round(angle + self.STEPS, 1)
            self.servo.angle = angle

            count = self.ser.in_waiting
            if count > 8:
                recv = self.ser.read(9)
                self.ser.reset_input_buffer()
                if recv[0] == 0x59 and recv[1] == 0x59:
                    distance = recv[2] + recv[3] * 256
                    strength = recv[4] + recv[5] * 256
                    print("(", distance, ",", strength, ")")
                    self.ser.reset_input_buffer()
                    good_data += 1
                else:
                    # self.something_went_wrong(count, recv)
                    distance = 0
                    # self.ser.reset_input_buffer()

                    err_count += 1
            else:
                distance = 0
            distance_arr.append(float(distance / 100))

    def publish_data(self, distance_arr):
        # print(distance_arr)

        scan = LaserScan()
        scan.header.frame_id = "lidar_link"
        # scan.header.frame_id = "map"
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.angle_min = math.radians(-90)
        scan.angle_max = math.radians(90)
        #        scan.angle_increment = float(math.radians(180.2) / len(distance_arr))
        scan.angle_increment = float(math.radians(180.4) / len(distance_arr))
        # scan.time_increment = float(self.SLEEP)
        scan.time_increment = 0.0
        scan.ranges = distance_arr

        self.publisher.publish(scan)

    def something_went_wrong(self, count, recv):
        # print("-----------------------------weird-----------------------------")
        # print("count:", count)
        """i = 0
        for elm in recv:
            print(str(i), elm, elm == 0x59, type(elm))
            i = i + 1"""


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
