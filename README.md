# Zeppelin
this project is in development
# Comands
Desktop:
    urdf+tf: ros2 launch sam_bot_description display.launch.py
    imu filter: ros2 launch imu_filter_madgwick imu_filter.launch.py
    nav: ros2 launch nav2_bringup navigation_launch.py
    slam: ros2 launch slam_toolbox online_async_launch.py
Raspberry Pi:
    ros2 launch mpu6050driver mpu6050driver_launch.py
    ./run
    sudo chmod 666 /dev/ttyS0
