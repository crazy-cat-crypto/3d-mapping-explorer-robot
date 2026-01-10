# 3D AUTONOMOUS EXPLORATION ROBOT 

---

## **EXECUTIVE SUMMARY**

A fully autonomous mobile robot that maps unknown environments, identifies exploration targets, and corrects navigation drift using 7 core robotics algorithms integrated across Arduino and Windows platforms. Uses trigonometry, sensor fusion, occupancy grids, frontier detection, and SLAM loop closure for complete autonomy.

---

## **SYSTEM ARCHITECTURE**

### **Hardware**
- **Arduino Uno** (real-time control, 100 Hz loop)
- **2× DC Motors + L298N driver** (propulsion with PWM control)
- **HC-SR04 Ultrasonic Sensor** (distance measurement, 10 Hz)
- **2× SG90 Servo Motors** (sensor scanning, 180° range)
- **2× 18650 Batteries** (3.7V, voltage-compensated)
- **NodeMCU ESP8266** (WiFi)
- **Windows PC** (map processing, SLAM correction)

### **Data Flow**
```
ARDUINO (Real-time)              WINDOWS (Processing)
├─ Read HC-SR04                 ├─ Receive CSV: time,x,y,θ,distance,angle
├─ Calculate odometry ①          ├─ Build occupancy grid ④
├─ Sensor geometry ②            ├─ Find frontiers ⑤
├─ Obstacle avoidance ③         ├─ SLAM correction ⑥
├─ Format + serialize ⑦         └─ Send waypoint back via serial ⑦
└─ 100 Hz loop          ↔ WiFi ↔       10 Hz loop
```

---

## ** ALGORITHMS IMPLEMENTED**

### **1 ODOMETRY + VOLTAGE COMPENSATION**
**What:** Measures distance traveled using motor PWM timing with automatic battery voltage compensation.

**Why Important:** As 18650 batteries discharge, motor speed at same PWM decreases. Without compensation, odometry drift accumulates. This algorithm adjusts PWM-to-distance mapping dynamically.

**Implementation:** https://github.com/ArminJo/PWMMotorControl/blob/master/src/CarPWMMotorControl.hpp -> ``setSpeedPWMCompensation``

**Expected Accuracy:** idk

---

### **2 SENSOR GEOMETRY**
**What:** Converts HC-SR04 distance + servo angle into 3D robot coordinates using trigonometry.

**Why Important:** Sensor provides distance (scalar), but robot needs to know WHERE obstacles are (vector). This calculates position relative to robot frame.

**Implementation:** idk

**Formula:**
```
x = distance × cos(angle)    // Lateral component
y = distance × sin(angle)    // Forward component
z = sensor_height            // Vertical component
```

**Real-World Use:** Enables sensor fusion for accurate obstacle mapping.

---

### **3 OBSTACLE AVOIDANCE AND PATH FINDING**
**What:** Reactive collision prevention—if distance < 20cm, stop and turn away. and try to move to a point as specified by frontier algorithm.

**Why Important:** Safety mechanism. Robot doesn't crash and actually explores.

**Implementation:** https://www.instructables.com/Arduino-Powered-Autonomous-Vehicle/ 

**Behavior:** Proven obstacle avoidance on Arduino Uno with L298N.

---

### **4 OCCUPANCY GRID MAPPING (PYTHON - Numpy, Matplotlib)**

**Note:** Python - Plotly, will be used for final interactive representation

**What:** Converts robot trajectory + sensor readings into 3D grid map (0=empty, 1=obstacle). 

**Why Important:** Creates visual representation of explored environment. Enables planning.

**Implementation:** idk

**Input:** CSV file (robot_data.csv)
```
timestamp, x, y, heading, distance, servo_angle
0.0,       0, 0, 0,       50,       45
0.1,       1, 0, 0,       48,       45
```

**Output:** `occupancy_grid.csv` + `visu.py` visualization

---

### **5 FRONTIER EXPLORATION (PYTHON)** | 
**What:** Identifies unexplored edges in occupancy grid and picks next exploration target and commands arduino to go there.

**Why Important:** Enables systematic area coverage instead of random wandering. Uses greedy frontier selection: closest edge with highest information gain.

**Implementation:** idk
**Algorithm:**
1. Find cells touching both explored and unexplored areas
2. Group nearby frontier cells
3. Score each cluster: `priority = 1/distance + information_gain`
4. Return highest-scoring frontier

**Result:** Autonomous target selection without manual intervention.

---

### **6 SLAM LOOP CLOSURE (PYTHON)**
**What:** Detects when robot returns to previously visited area and corrects accumulated position drift.

**Why Important:** Odometry error accumulates over time (~1-2m after 10min exploration). Loop closure fixes this by recognizing revisited locations and recalculating entire trajectory.

**Implementation:** idk

**Input:** Robot trajectory (x,y over time) + occupancy grid snapshots

**Output:** Corrected trajectory with drift removed

**Example:** Robot thinks it's at (5.2, 4.8) but grid matches past scan at (5.0, 5.0) → corrects entire history.

---

### **7 SERIAL INTERFACE (Arduino - Wifi -> Python - Flask)**
**What:** Bidirectional Arduino ↔ Windows communication.

**Why Important:** Arduino collects real-time sensor data, Windows performs heavy computation (mapping, SLAM,frontier exploration). Serial bridge enables closed-loop autonomy.

**Implementation:** idk

**Format:** CSV lines (simple, robust, human-readable)

**Example Flow:**
```
Arduino → Windows: "0.5,1.2,2.1,0.5,50,45"   (sensor reading)
Windows → Arduino: "10.5,8.2"                 (frontier goal)
```

---
