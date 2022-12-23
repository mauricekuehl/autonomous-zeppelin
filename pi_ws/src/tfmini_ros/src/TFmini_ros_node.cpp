#include "tfmini_ros/TFmini.h"
#include <chrono>
#include <thread>
#include <iostream>

static int degree = 0;

void update_degree_async()
{
  degree += 1;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

int main(int argc, char **argv)
{
  std::cout << "start running main" << std::endl;

  // ros::init(argc, argv, "tfmini_ros_node");
  // ros::NodeHandle nh("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tfmini_ros_node");

  std::string id = "TFmini";
  std::string portName = "/dev/ttyS0";
  int baud_rate = 115200;
  benewake::TFmini *tfmini_obj;

  // ros::Rate r(60);
  rclcpp::Rate r(60);

  std::cout << "start running 1" << std::endl;

  // nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  // nh.param("baud_rate", baud_rate, 115200);
  // node.param("serial_port", portName, std::string("/dev/ttyS0"));
  // node.param("baud_rate", baud_rate, 115200);

  tfmini_obj = new benewake::TFmini(portName, baud_rate);

  // ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(id, 1000, true);
  auto pub_range = node->create_publisher<sensor_msgs::msg::Range>(id,
                                                                   1000);
  std::cout << "start running 2" << std::endl;

  // sensor_msgs::Range TFmini_range;
  sensor_msgs::msg::Range TFmini_range;
  // TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;

  TFmini_range.radiation_type = sensor_msgs::msg::Range::INFRARED;

  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;
  float dist = 0;
  // ROS_INFO_STREAM("Start processing ...");

  // run update_degree_async() in a separate thread
  // std::thread t(update_degree_async);
  // std::thread t1(update_degree_async);
  // std::thread worker(update_degree_async);

  // while (ros::ok())
  std::cout << "start running loop" << std::endl;

  while (rclcpp::ok())
  {
    std::cout << "degree: " << degree << std::endl;
    // ros::spinOnce();
    dist = tfmini_obj->getDist();
    if (dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist;
      // TFmini_range.header.stamp = ros::Time::now();
      TFmini_range.header.stamp = node->now();
      pub_range->publish(TFmini_range);
    }
    else if (dist == -1.0)
    {
      // ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      break;
    }
    else if (dist == 0.0)
    {
      // ROS_ERROR_STREAM("Data validation error!");
    }
    // r.sleep();
  }

  tfmini_obj->closePort();
}
