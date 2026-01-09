# Algorithm Best Repositories

## Algorithm Implementations for Robotics & Mapping

| # | Algorithm | Best Repository | One-Line Explanation |
|---|-----------|-----------------|----------------------|
| 1 | Motor Control with Voltage Compensation | ArminJo/PWMMotorControl | Arduino library for L298N motors with PWM speed adjustment, encoder support, and 2-motor car control (already attached). |
| 2 | Obstacle Avoidance (Ultrasonic) | stheophil/MappingRover | Arduino + C++ Windows controller: reads HC-SR04 sonar, detects obstacles, sends movement commands back to Arduino via Bluetooth. |
| 3 | Odometry (Encoder Tracking) | lucimobility/encoder-to-odom-library | C++ library converts differential-drive encoder pulses into accurate position, velocity, and distance (Windows-compatible). |
| 4 | Occupancy Grid Mapping | farzingkh/Occupancy-Grid-Mapping | C++ Windows app: takes Arduino odometry + ultrasonic data, builds 2D occupancy grid with visualization. |
| 5 | Frontier Exploration | stheophil/MappingRover(robot_controller.cpp) | Same repo: implements edge-following strategy to explore all unmapped borders in occupancy grid. |
| 6 | SLAM Loop Closure | stheophil/MappingRover(robot_controller.cpp) | Matches occupancy grid snapshots; detects when robot revisits location by comparing sonar patterns. |
