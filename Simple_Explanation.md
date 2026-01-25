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
├─ Read HC-SR04                 ├─ Receive CSV: time,x,y,θ,distance,angle
├─ Calculate odometry           ├─ SLAM correction 
├─ Sensor geometry              ├─ Build occupancy grid 
├─ Waypoint algorithm           ├─ Find frontiers
├─ Format + serialize           └─ Send waypoint back via serial 
└─  loop               ↔ WiFi ↔        loop
```

---

## ** ALGORITHMS IMPLEMENTED **

### **1 ODOMETRY (Microcontroller)**
**What:** Measures distance traveled using signals from wheels encoders.


**Processing:** Calculates (x,y) position of the robot considerating for how much speed and time did each wheels moved 

**Output:** To SENSOR GEOMETRY

**Implementation:** https://github.com/tum-phoenix/drive_ros_localize_wheel_odometry

---

### **2 SENSOR GEOMETRY (Microcontroller)**

**What:** Converts HC-SR04 distance + servo angle into 3D coordinates using trigonometry.

**Input:** From ODOMETRY + Ultrasonic sensor + servo angles

**Processing:** Uses trignometry to turn an object postion to (x,y,z)

**Output:** From wifi module transfers (x,y,z) to SERIAL INTERFACE

**Why Important:** Sensor provides distance (scalar), but robot needs to know WHERE obstacles are (vector). This calculates position relative to robot frame.

**Implementation:** https://gist.github.com/PCJohn/fa94b020d8710cabe29c
                    https://www.dropbox.com/scl/fi/dfkr6ksiiiqni8ukxzioq/3D_SCANNER.pdf?rlkey=dpddjdihd21prl44juvf1fm0o&e=1&dl=0  **(NOTE: They donot move)**

**Formula:**
```
x = distance × cos(angle)    // Lateral component
y = distance × sin(angle)    // Forward component
z = sensor_height            // Vertical component
```

---

### **3 OBSTACLE AVOIDANCE AND PATH FINDING (Microsontroller)**
**What:** Reactive collision prevention—if distance < 20cm, stop and turn away and while trying to move to a point as specified by frontier exploration algorithm.

**Why Important:** Safety mechanism. Robot doesn't crash and actually explores.

**Implementation:** https://www.instructables.com/Arduino-Powered-Autonomous-Vehicle/ **(NOTE: It doesnot create 3D model)**

---
### **4 SERIAL INTERFACE (Microsontroller - Wifi <-> Python - Flask)**

**What:** Bidirectional Arduino ↔ Windows communication.

**Input:** From SENSOR GEOMETRY

**Processing:** Takes hundreds of (x,y,z) value from arduino`s wifi module and converts it to a file.

**Output:** The file is accessed by SLAM.

**Why Important:** Arduino collects real-time sensor data, Windows performs heavy computation (mapping, SLAM,frontier exploration). Serial bridge enables closed-loop autonomy.

**Implementation:** https://techtutorialsx.com/2017/01/08/esp8266-posting-json-data-to-a-flask-server-on-the-cloud/

**Format:** CSV lines (simple, robust, human-readable)

**Example Flow:**
```
Arduino → Windows: "0.5,1.2,2.1,0.5,50,45"   (sensor reading)
Windows → Arduino: "10.5,8.2"                 (frontier goal)
```
---
### **5 SLAM LOOP CLOSURE (PYTHON)**

**What:** Step where the robot recognizes it has returned to a previously visited place and adjusts its entire estimated path to remove accumulated drift.

**Input:** From File made by SERIAL INTERFACE i.e Robot trajectory (x,y over time) + occupancy grid snapshots

**Processing:** Detects if robot has returned to previously visited area Then corrects accumulated position drift.

**Output:** Corrected trajectory with drift removed or if previous position is not detected forwards the File received from SERIAL INTERFACE to OCCUPANCY GRID MAPPING.

**Example:** Robot thinks it's at (5.2, 4.8) but grid matches past scan at (5.0, 5.0) → corrects entire history.

**Why Important:** Odometry error accumulates over time (~1-2m after 10min exploration). Loop closure fixes this by recognizing revisited locations and recalculating entire trajectory.

**Implementation:** https://github.com/simondlevy/BreezySLAM/blob/master/python/breezyslam/algorithms.py

---
### **6 OCCUPANCY GRID MAPPING (PYTHON - Numpy, Matplotlib)**

**Note:** Python - Plotly, will be used for final interactive 3D representation taking direct input from serial interface.

**What:** Converts SLAM LOOP CLOSURE output into 2D grid map(ignoring Z axis) then sends the map to FRONTIER EXPLORATION.(0=empty, 1=obstacle). 

**Why Important:** Creates visual representation of explored environment. Enables planning.

**Implementation:** https://github.com/AtsushiSakai/PythonRobotics/blob/master/Mapping/lidar_to_grid_map/lidar_to_grid_map.py **(NOTE: It supposes lidar as input)**

**Input:** CSV file (robot_data.csv)
```
timestamp, x, y, heading, distance, servo_angle
0.0,       0, 0, 0,       50,       45
0.1,       1, 0, 0,       48,       45
```

**Output:** `occupancy_grid.csv` + `visu.py` visualization

---

### **7 FRONTIER EXPLORATION (PYTHON)**
**What:** Identifies unexplored edges in occupancy grid created and picks next exploration target and commands arduino to go there via SERIAL INTERFACE.

**Why Important:** Enables systematic area coverage instead of random wandering. Uses greedy frontier selection: closest edge with highest information gain.

**Implementation:** create a simle algorithm by ourselves.

**Algorithm:**
1. Find cells touching both explored and unexplored areas
2. Group nearby frontier cells
3. Score each cluster: `priority = 1/distance + information_gain`
4. Return highest-scoring frontier

**Result:** Autonomous target selection without manual intervention.

---




