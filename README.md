# Requirements

- ROS 2
  
- Python 3.x

- Arduino (for uploading code to the IMU)
  
- rclpy, sensor_msgs, geometry_msgs, pyserial

## Build & Run
    
    ### Connect the IMU via USB and ensure it sends data in JSON format.
    
    ### Build:

    git clone 
    
    cd 
    
    colcon build
    
    source install/setup.bash
    
    ### Launch:
    
    ros2 launch imu_serial serial_imu_node.py

    rviz2
    
## Simulation: https://youtu.be/xpN31UG7EPU?si=Jq8nFUgI_UAlH3RD
