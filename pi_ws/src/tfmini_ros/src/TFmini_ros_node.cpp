#include "tfmini_ros/TFmini.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "/usr/local/include/pigpio.h"
#include <functional>

const float RPM = 1.0f;
const float Hz = 1000.0f;
const int len_arr = static_cast<int>(Hz / RPM * 2);

const float PI = 3.14159265358979f;

const int HALL_PIN = 18;
const int SERVO_PIN = 17;
const int MIN_WIDTH = 1280;
const int MAX_WIDTH = 1720;
const int MID_WIDTH = 1500;
const int SPEED = 1630;

int run = 1;

int in_mid = 0;
uint32_t lastTick = 0;
uint32_t last_time = 0;

sensor_msgs::msg::LaserScan TFmini_range;
std::vector<float> v;
rclcpp::Node::SharedPtr node;
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_range;

bool calibration = false;
int turns = 0;
int readings = 0;
int pref_size = 0;

void alert(int gpio, int level, uint32_t tick)
{
  if (level == 1)
  {
    uint32_t time_dif = tick - lastTick;

    if (700 < time_dif && time_dif > 800)
    {
      in_mid = 1;
    }
    else if (in_mid && last_time > 900 && time_dif < 200)
    {
      if (calibration)
      {
        std::cout << "size = " << v.size() << std::endl;
        int dif = v.size() - pref_size;
        std::cout << "dif = " << dif << std::endl;

        if (dif > 0)
        {
          // remove last elements
          v.erase(v.end() - dif, v.end());
        }
        else if (dif < 0)
        {
          // add 0 to the end
          std::vector<float> temp(-dif, 0);
          v.insert(v.end(), temp.begin(), temp.end());
        }
        TFmini_range.ranges = v;
        TFmini_range.angle_increment = 2 * PI / v.size();
        TFmini_range.header.stamp = node->now();
        pub_range->publish(TFmini_range);
        std::cout << "final size = " << v.size() << std::endl;
        // for (float i : v)
        // {
        //   std::cout << i << " ";
        //   if (i == 0)
        //     std::cout << std::endl;
        // }
        // std::cout << std::endl;
        v.clear();
        in_mid = 0;
        printf("turn %d %d \n", last_time, time_dif);
        std::cout << std::endl;
      }
      else
      {
        int starting_rounds = 10;
        int messure_rounds = 10;

        turns++;
        std::cout << "turns = " << turns << std::endl;
        std::cout << "size = " << v.size() << std::endl;
        int bad_reading = 0;
        for (int i = 0; i < v.size(); i++)
        {
          if (v[i] == 0.0)
            bad_reading++;
        }
        std::cout << "bad readings = " << bad_reading << std::endl;
        std::cout << "readings = " << readings << std::endl;
        if (turns == starting_rounds + messure_rounds)
        {
          calibration = true;
          turns -= starting_rounds;
          readings += v.size();
          pref_size = (int)(readings / turns);
          std::cout << "calibration done" << std::endl;
          std::cout << "readings = " << readings << std::endl;
          std::cout << "turns = " << turns << std::endl;
          std::cout << "readings/turns = " << pref_size << std::endl;
        }
        else if (turns > starting_rounds)
        {
          readings += v.size();

          std::cout << "start counting" << std::endl;
        }
        v.clear();
        in_mid = 0;
        std::cout << std::endl;
      }
    }
    last_time = time_dif;
  }
  else
  {
    lastTick = tick;
  }
};

void stop(int i)
{
  run = i;
}

int main(int argc, char **argv)
{
  std::cout << "start running main" << std::endl;

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("tfmini_ros_node");

  std::string id = "scan";
  std::string portName = "/dev/ttyS0";
  int baud_rate = 115200;
  benewake::TFmini *tfmini_obj;

  rclcpp::Rate r(60);

  std::cout << "start running 1" << std::endl;

  tfmini_obj = new benewake::TFmini(portName, baud_rate);

  pub_range = node->create_publisher<sensor_msgs::msg::LaserScan>(id, 1000);
  std::cout << "start running 2" << std::endl;

  TFmini_range.angle_min = 0.0;
  TFmini_range.angle_max = 2 * PI;
  TFmini_range.range_min = 0.3;
  TFmini_range.range_max = 12;
  TFmini_range.header.frame_id = "base_link";

  float dist = 0;

  if (gpioInitialise() < 0)
    return 1;
  gpioSetMode(HALL_PIN, PI_INPUT);
  gpioSetPullUpDown(HALL_PIN, PI_PUD_UP);
  gpioSetAlertFunc(HALL_PIN, alert);

  gpioSetSignalFunc(SIGINT, stop);
  gpioServo(SERVO_PIN, SPEED);
  time_sleep(1);

  std::cout << "start running loop" << std::endl;
  while (rclcpp::ok())
  {
    dist = tfmini_obj->getDist();
    v.push_back(dist);
    if (dist == -1.0)
    {
      std::cout << "Failed to read data. TFmini ros node stopped!" << std::endl;
      break;
    }
    else if (dist == 0.0)
    {
      std::cout << "Data validation error!" << std::endl;
    }
  }

  gpioServo(SERVO_PIN, 0);
  gpioTerminate();
  tfmini_obj->closePort();
  return 0;
}
