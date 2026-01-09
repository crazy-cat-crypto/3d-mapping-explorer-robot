# ROBOT ALGORITHMS - COMPLETE GUIDE (Normal User Explanation + File Locations)

---

## **MASTER REFERENCE TABLE**

| # | Algorithm Name | What It Does (Simple) | GitHub Link | Key File | Input | Output |
|---|---|---|---|---|---|---|
| **1** | **Odometry + Voltage** | Tracks how far robot moved, adjusts for weak battery | https://github.com/ArminJo/PWMMotorControl | `src/CarPWMMotorControl.hpp` | Motor PWM signal | Distance traveled (mm) |
| **2** | **Sensor Geometry** | Converts distance + angle to X,Y,Z coordinates | https://www.instructables.com/Ultrasound-Sensor-2D-Tracking-With-Arduino/ | Tutorial page (code section) | HC-SR04 distance + servo angle | Object location (x,y,z) |
| **3** | **Obstacle Avoidance** | Immediate reflex: if too close, stop and turn | https://github.com/JanithDisanayake/Arduino-Robot | `myRobotX.ino` | HC-SR04 distance reading | Motor control (forward/backward/turn) |
| **4** | **Occupancy Grid** | Creates checkerboard map: empty vs obstacles | https://github.com/omerhalid/occupancy_grid_mobile_robot_cpp | `src/main.cpp` | CSV: robot position + sensor distance | `occupancy_grid.csv` (grid map) |
| **5** | **Frontier Exploration** | Finds unexplored edges, picks next target | https://github.com/HanwenCao/Frontier_Exploration | `my_frontier/scripts/demo.py` | Occupancy grid map | Next waypoint (fx, fy) to explore |
| **6** | **SLAM Loop Closure** | Recognizes old places, fixes position drift | https://github.com/manonkok/1d-magnetic-field-slam/ | `runSLAM.m` | Odometry + occupancy grid snapshots | Corrected trajectory (x,y error fixed) |
| **7** | **Serial Interface** | Arduino ↔ Windows communication | https://www.allaboutcircuits.com/technical-articles/csharp-windows-application-for-arduino/ | Tutorial (C# code shown) | Arduino data (serial) | CSV file saved on Windows |

---

## **SIMPLE EXPLANATIONS (One Sentence Each)**

### **Algorithm #1: Odometry + Voltage Compensation**
**"I'm measuring how far the robot moved by timing the motors, but I adjust my math when the battery gets tired so the numbers stay accurate."**

- **Human analogy:** You're measuring rope by counting hand-spans, but your hands get shorter when tired (battery dies = voltage drops = motors slower), so you compensate.
- **Device:** Arduino Uno
- **Main library file:** `src/CarPWMMotorControl.hpp`
- **Best example:** `examples/Start/Start.ino`
- **Key function:** `goDistanceMillimeter(speed, distance_mm, direction)`

---

### **Algorithm #2: Sensor Geometry**
**"My sensor says 'something is 50cm away and I'm pointing 45 degrees' so I calculate 'it's 35cm right and 35cm forward' using triangle math."**

- **Human analogy:** Your friend shines a flashlight at something 50 meters away while standing at a 45° angle. You use geometry to figure out WHERE the thing actually is.
- **Device:** Arduino Uno + HC-SR04 + Servo
- **This is NOT code to download** - it's a **teaching tutorial** showing you the formula
- **Key formula:**
  ```
  x = distance × cos(angle)
  y = distance × sin(angle)
  ```
- **Website has code example** in the "Code" section that you copy-paste

---

### **Algorithm #3: Obstacle Avoidance**
**"If something is closer than 20cm, I panic: STOP! Look around! Find the clearest direction! GO THAT WAY!"**

- **Human analogy:** Someone suddenly steps in front of you in a hallway. You freeze, look left/right quickly, then sidestep around them.
- **Device:** Arduino Uno + HC-SR04 + L298N + Motors
- **Main file:** `myRobotX.ino`
- **Logic:** Read distance → If distance < threshold → Scan with servo → Find max distance direction → Execute movement
- **No planning, just reaction**

---

### **Algorithm #4: Occupancy Grid Mapping**
**"I take all your robot's sensor readings and paint a map: white squares = empty, black squares = obstacles."**

- **Human analogy:** You walk through a building with a thermal camera and mark a grid on paper as you go - "this square had a wall, this one didn't."
- **Device:** Windows PC (runs C++ program)
- **Main file:** `src/main.cpp`
- **Input:** CSV file with robot position + distance readings
- **Output:** CSV file with grid (0 = free, 1 = blocked)
- **Visualization:** Python script `visu.py` creates colorful map image

---

### **Algorithm #5: Frontier Exploration**
**"I look at the map you made and find where it says 'not explored yet' - then I pick the closest such edge and say 'go there next!'"**

- **Human analogy:** You've explored half a cave. I look at your map and say "the border between explored and unexplored is over there - explore that next."
- **Device:** Windows PC (Python script)
- **Main file:** `my_frontier/scripts/demo.py`
- **Input:** Occupancy grid map
- **Output:** Frontier cells (list of unexplored edges)
- **Scoring:** Distance + information gain = "usefulness score"

---

### **Algorithm #6: SLAM Loop Closure**
**"Wait - this place LOOKS familiar! I think I've been here before. So let me go back and FIX ALL MY PAST MISTAKES."**

- **Human analogy:** You've been walking in circles making small navigation errors. Suddenly you see a landmark and realize "I've been here 10 minutes ago!" Now you can fix your entire route.
- **Device:** Windows PC (MATLAB or C++)
- **Main file:** `runSLAM.m` (MATLAB) or `src/magSLAMwithLoopClosures.m`
- **Input:** Robot trajectory (x, y over time) + sensor snapshots
- **Output:** Corrected trajectory (errors removed)
- **Method:** Extended Kalman Filter (EKF) + loop detection

---

### **Algorithm #7: Serial Interface**
**"Arduino and Windows PC talk to each other through a USB cable at very high speed (115200 bits/second)."**

- **Human analogy:** Text messaging between robot and computer - robot sends "sensor readings", computer sends back "where to go next"
- **Devices:** Arduino Uno + Windows PC connected by USB
- **Speed:** 115200 baud (very fast for serial)
- **Format:** CSV lines (comma-separated values)
- **Tutorial site:** https://www.allaboutcircuits.com/technical-articles/csharp-windows-application-for-arduino/

---

## **EXACT FILE LOCATIONS IN GITHUB REPOS**

### **Algorithm #1: PWMMotorControl**
```
https://github.com/ArminJo/PWMMotorControl/
├── src/
│   ├── PWMDcMotor.hpp              ← Single motor control
│   ├── CarPWMMotorControl.hpp      ← Two motors for your car ⭐
│   ├── EncoderMotor.hpp            ← Distance tracking
│   └── ...
├── examples/
│   ├── Start/                      ← BEST STARTING POINT ⭐
│   │   ├── Start.ino               ← Open this file first
│   │   ├── RobotCarPinDefinitionsAndMore.h  ← Edit pin numbers here
│   │   └── ...
│   ├── Square/Square.ino           ← Movement example
│   └── RobotCarBlueDisplay/        ← Full featured example
└── README.md                       ← Documentation
```

### **Algorithm #2: Sensor Geometry**
```
https://www.instructables.com/Ultrasound-Sensor-2D-Tracking-With-Arduino/
(Not a GitHub repo - it's a tutorial website)

Navigate to:
→ Step-by-step instructions
→ Scroll to "Code" section
→ Copy the Arduino sketch code
→ Key part shows:
   x = distance * cos(angle)
   y = distance * sin(angle)
```

### **Algorithm #3: Arduino-Robot**
```
https://github.com/JanithDisanayake/Arduino-Robot/
├── myRobotX.ino                 ← MAIN FILE - Use this one! ⭐
├── myRobot.ino                  ← Older version
├── newlineFollower.ino          ← Different robot, ignore
├── Schematics/                  ← Wiring diagrams (ignore)
└── README.md                    ← Explanation
```

### **Algorithm #4: Occupancy Grid**
```
https://github.com/omerhalid/occupancy_grid_mobile_robot_cpp/
├── src/
│   ├── main.cpp                 ← Main program ⭐
│   ├── dataProcessor.hpp        ← Reads CSV file
│   ├── dataProcessor.cpp
│   ├── occupancyGrid.hpp        ← Creates grid
│   ├── occupancyGrid.cpp
│   ├── sensor.hpp               ← Sensor properties
│   └── position.hpp             ← Data structures
├── data/
│   └── robot1.csv               ← Example input
├── CMakeLists.txt               ← Build instructions
├── visu.py                      ← Visualization script ⭐
└── README.md
```

### **Algorithm #5: Frontier Exploration**
```
https://github.com/HanwenCao/Frontier_Exploration/
├── my_frontier/
│   └── scripts/
│       ├── demo.py              ← Main demo ⭐
│       ├── frontier_detection.py ← Core algorithm
│       └── ...
├── nav_demos/
│   └── launch/
│       ├── gazebo.launch        ← Simulator setup
│       ├── nav_mapping.launch   ← Mapping
│       └── demo.launch          ← Launcher
└── README.md

OR simpler version:
https://github.com/adrian-soch/frontier_exploration/
├── frontier_exploration/
│   ├── src/
│   │   └── exploration_node.cpp ← Core code
│   └── scripts/
└── README.md
```

### **Algorithm #6: SLAM Loop Closure**
```
https://github.com/manonkok/1d-magnetic-field-slam/
├── src/
│   ├── magSLAMwithLoopClosures.m  ← Main SLAM ⭐
│   ├── run_filter_from_scratch.m  ← EKF implementation
│   ├── dynamics.m                 ← Motion model
│   └── ...
├── tools/
│   ├── prepareData.m              ← Prepare input
│   ├── makePlots.m                ← Visualize output
│   └── rotation functions (rotx, roty, rotz, quat2eul)
├── data/                          ← Example datasets
├── runSLAM.m                      ← Main file to run ⭐
└── README.md
```

---

## **INTEGRATION PIPELINE (How They Connect)**

```
ARDUINO (Real-time loop ~100Hz)
├─ Read HC-SR04 distance
├─ Read servo angle
├─ Calculate odometry (Algorithm #1)
├─ Calculate coordinates (Algorithm #2)
├─ Check for obstacles (Algorithm #3)
├─ Format as CSV: time,x,y,theta,distance,angle
└─ Send via Serial to Windows PC

                    ↓ [USB Cable at 115200 baud]

WINDOWS (Background processing ~10Hz)
├─ Receive CSV lines
├─ Save to file: robot_data.csv
├─ Run Algorithm #4 (Occupancy Grid)
│  └─ Output: occupancy_grid.csv
├─ Run Algorithm #5 (Frontier Exploration)
│  └─ Output: frontier_waypoints.txt
├─ Run Algorithm #6 (SLAM Loop Closure)
│  └─ Output: corrected_trajectory.txt
└─ Send next frontier goal back via Serial

                    ↓ [USB Cable]

ARDUINO (Receives new target)
├─ Parse frontier coordinates
├─ Plan path (simple line-to-point)
├─ Execute movement to frontier
└─ Loop back to collect more data
```

---
