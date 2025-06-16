## Build & Run
    
  1. Connect the IMU via USB.
    
  2. Build:

    git clone https://github.com/ichsanyudika/Razor-IMU-9dof_RVIZ2.git
    
    cd ~/Razor-IMU-9dof_RVIZ2
    
    colcon build
    
    source install/setup.bash
    
  3. Launch:
    
    ros2 launch imu_serial serial_imu_node.py

    rviz2
    
## Simulation: https://youtu.be/xpN31UG7EPU?si=Jq8nFUgI_UAlH3RD
