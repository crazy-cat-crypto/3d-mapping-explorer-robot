# 3D AUTONOMOUS EXPLORATION ROBOT 

---

## **SYSTEM ARCHITECTURE**

### **Hardware**
- **2× DC Motors with wheels encoders + L298N driver** 
- **HC-SR04 Ultrasonic Sensor** (distance measurement, 10 Hz)
- **2× SG90 Servo Motors with servo brackets** (sensor scanning, 180° range)
- **3× 18650 Batteries** (3.7V)
- **NodeMCU ESP8266** (WiFi and microcontroller)
- **Buck Converter** (To convert 12V to 5V)
- **Windows PC** (map processing, SLAM correction,frontier algorithm)

### **Data Flow**
```
Microsontroller (Real-time)              WINDOWS (Processing)
├─ Read Sensors                 ├─ Receive CSV: time,x,y,θ,distance,pan_angle,tilt_angle
├─ Calculate odometry           ├─ SLAM correction 
|                               ├─ Build occupancy grid 
├─ object-avoidance algorithm   ├─ Find frontiers
├─ Waypoint algorithm           ├─ Send waypoint back via wifi
|                               └─3D visualization 
└─  loop               ↔ WiFi ↔        loop
```

---

## ** ALGORITHMS IMPLEMENTED **

### **1 ODOMETRY AND SENSOR READING(Microcontroller) **
**What:** Measures distance traveled using signals from wheels encoders and receive distance from ultrasonic sensor and angles from servo motors.

**Processing:** Calculates (x,y) position of the robot considerating for how much speed and time did each wheels moved.

**Output:** (timestamp,x,y,\(\theta \),distance,pan,tilt) to COMMUNICATION ALGORITHM.

**Note:** \(\theta \) = absolute direction its facing eg north=0, east=90.

**Implementation:** https://github.com/purwar-lab/DeadReckoning-library , https://github.com/jshaw/NewPingESP8266

---

### **2 OBSTACLE AVOIDANCE AND PATH FINDING (Microsontroller)**
**What:** Reactive collision prevention—if distance < 20cm, stop and turn away and while trying to move to a point as specified by frontier exploration algorithm.

**Why Important:** Safety mechanism. Robot doesn't crash and actually explores.

**Implementation:** https://www.instructables.com/Arduino-Powered-Autonomous-Vehicle/ **(NOTE: It doesnot create 3D model)**

---
### **3 COMMUNICATION ALGORITHM (Microsontroller - Wifi <-> Python - Flask)**

**What:** Bidirectional Microcontroller ↔ Windows communication.

**Input:** From ODOMETRY AND SENSOR READING

**Processing:** Takes tens of (timestamp,x,y,\(\theta \),distance,pan,tilt) value from microcontroller in json format and converts it to a csv file.

**Output:** The file is accessed by SLAM.

**Implementation:** https://techtutorialsx.com/2017/01/08/esp8266-posting-json-data-to-a-flask-server-on-the-cloud/

**Note:** Also input from frontier algorithm(x,y) and send it to microprocessor.

---
### **4 SLAM (PYTHON)**

**What:** Step where the robot recognizes it has returned to a previously visited place and adjusts its entire estimated path to remove accumulated drift.

**Input:** From File made by COMMUNICATION ALGORITHM

**Output:** Corrected trajectory with drift removed or if previous position is not detected forwards the File received from COMMUNICATION ALGORITHM to OCCUPANCY GRID MAPPING.

**Note:** A algotithm to change the 3D data as well should be implemented as slam only takes 2D data and corrects it.

**Example:** Robot thinks it's at (5.2, 4.8) but grid matches past scan at (5.0, 5.0) → corrects entire history.

**Why Important:** Odometry error accumulates over time (~1-2m after 10min exploration). Loop closure fixes this by recognizing revisited locations and recalculating entire trajectory.

**Implementation:** https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/particle_filter/particle_filter.py

---
### **5 OCCUPANCY GRID MAPPING AND FRONTIER ALGORITHM (PYTHON - Numpy, Matplotlib)**

**What:** Converts SLAM corrected file into 2D grid map then uses greedy frontier selection: closest edge with highest information gain.. 

**Why Important:** Enables systematic area coverage instead of random wandering. Autonomous target selection without manual intervention.

**Implementation:** https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py **(NOTE: It supposes lidar as input)**
                    for Frontier: to write a simple greedy algorithm


---

### **6 3D REPRESENTATION AND GEOMETRY (PYTHON - Plotly)**
**What:** In Last, SLAM corrected file(with also tilt angle corrected) is used to calculate exact object (x,y,z) using trignometry and then the data is displayed.

---




