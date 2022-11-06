#!/usr/bin/python3
# import subprocess

import serial
import RPi.GPIO as GPIO
import time


# Lidar
SERIAL_PATH = "/dev/ttyS0"
COM_RATE = 115200


class Scanner:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PATH, COM_RATE)
        self.err_count = 1
        self.good_data = 0
        self.turn_time = 0
        self.sum_good = 1
        self.sum_bad = 1
        self.start_time = time.time()

    def run(self):
        print("run")
        while True:

            if self.ser.is_open == False:
                self.ser.open()

            distance_arr = []
            distance = 0
            if time.time() - self.turn_time > 1:
                print(
                    "bad:",
                    self.err_count,
                    "good:",
                    self.good_data,
                    "ratio:",
                    self.good_data / self.err_count,
                )
                self.sum_good += self.good_data
                self.sum_bad += self.err_count
                print(
                    "sum good:",
                    self.sum_good,
                    "sum bad:",
                    self.sum_bad,
                    "ratio:",
                    self.sum_good / self.sum_bad,
                    "good per sek:",
                    self.sum_good / (time.time() - self.start_time),
                )
                self.turn_time = time.time()
                self.good_data = 0
                self.err_count = 1

            count = self.ser.in_waiting
            if count > 8:
                recv = self.ser.read(9)
                self.ser.reset_input_buffer()
                if recv[0] == 0x59 and recv[1] == 0x59:
                    distance = recv[2] + recv[3] * 256
                    # strength = recv[4] + recv[5] * 256
                    # print("(", distance, ",", strength, ")")
                    self.ser.reset_input_buffer()
                    self.good_data += 1
                else:
                    self.something_went_wrong()
                    distance = 0
                distance_arr.append(float(distance / 100))

    def something_went_wrong(self):
        self.err_count += 1

    def stop(self):
        print("stop")
        self.ser.close()


def main():
    # res = subprocess.run(["sudo", "chmod", "666", "/dev/ttyS0"], capture_output=True)
    scanner = Scanner()
    try:
        scanner.run()
    except KeyboardInterrupt:
        print("ctrl+c")
    except Exception as e:
        print(e)
        print("ERROR")
    finally:
        scanner.stop()


if __name__ == "__main__":
    main()
