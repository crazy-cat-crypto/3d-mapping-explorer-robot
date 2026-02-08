#include <NewPing.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>


// --- WIFI SETTINGS ---
const char* ssid = "oldtaj_2.5";
const char* password = "9821976791@sad";
String server = "http://192.168.1.94:5000";


// --- 3D SCANNING SETTINGS ---
int tilt_angles[] = {60, 90, 120}; // Down, Straight, Up
int scan_start_pan = 25;
int scan_end_pan = 155;
int pan_step = 20; 

// Motor
const int IN1 = D5; //  Motor A Forward
const int IN2 = D6; //  Motor A Backward
const int ENA = D8; //  Motor A Speed (PWM)
const int IN4 = D7; //   Motor B Backward
const int IN3 = D0; //   Motor B Forward
const int ENB = D1;  //  Motor B Speed (PWM)
const int pan_servo_pin = D3; //   Pan Motor
const int tilt_servo_pin = D9;  //  Tilt Motor

// Ultra Sonic
const int TRIG_PIN = D4;  
const int ECHO_PIN = D2;  
const int max_distance = 200;

// --- SETTINGS ---
float move_speed = 0.01275; // How many cm per ms the robot moves
float turn_speed = 0.06; // How many degrees it turns per ms
float trim = 0.9; // Arc of slow wheel / Arc of fast wheel ; left wheel is slower.
bool is_moving=false;

// Odometry
float x = 0.0;
float y = 0.0;
float theta = 0.0; 
unsigned long last_time = 0;
int target_x = 0; // Where we want to go (received from server)
int target_y = 0;


NewPing sonar(TRIG_PIN, ECHO_PIN, max_distance);
Servo pan;
Servo tilt;
WiFiClient client;
HTTPClient http;
HTTPClient display_http;



void setup() {
  // Initialize all pins as OUTPUT
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(tilt_servo_pin, OUTPUT); pinMode(pan_servo_pin, OUTPUT); pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT);
  pan.attach(pan_servo_pin, 550, 2550); tilt.attach(tilt_servo_pin, 500, 2300);

  


  WiFi.begin(ssid, password);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  display("Wifi connected");
  last_time = millis();
  forward(200);
  delay(1000);
  brake();
}  

void loop() {
  reconnect();

  // 1. SYNC & SCAN (The "Think" Phase)
  // Stop everything to get a perfect map and the next target from Python
  brake();
  display("Mapping...");
  scan_and_send(); 

  // 2. AIM (The "Orient" Phase)
  float dx = target_x - x;
  float dy = target_y - y;
  float distance_to_target = sqrt(dx*dx + dy*dy);
  
  // Calculate turn
  float target_theta = atan2(dy, dx) * 57.295; 
  float turn_needed = target_theta - theta;

  // Shortest path turn logic (-180 to 180)
  while (turn_needed > 180) turn_needed -= 360;
  while (turn_needed < -180) turn_needed += 360;

  if (abs(turn_needed) > 10) {
    display("Aiming...");
    rotate(turn_needed);
    delay(100); 
  }

  // 3. PULSE (The "Move" Phase)
  // Instead of driving the whole way, we move in 20cm "pulses"
  // This keeps odometry errors from building up
  if (distance_to_target > 5) {
    display("Moving...");
    
    unsigned long pulse_start = millis();
    is_moving = true;

    // Drive for max 1.5 seconds OR until obstacle
    while (millis() - pulse_start < 1500) {
      forward(170);
      
      // Check for obstacles 10 times a second
      int obs = sonar.ping_cm();
      if (obs > 0 && obs < 20) {
        display("Obstacle!");
        break; // Exit the pulse immediately
      }
      delay(100); 
    }
    brake(); // Stop to recalculate everything
  }

  delay(200); // Short breather for the motors
}

// --- MOVEMENT FUNCTIONS ---

void forward(int speed) {
  update_position();
  is_moving=true;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed* trim);
  analogWrite(ENB, speed ); // Apply trim to straight line
}

void rotate(float degrees) {
  update_position();
  is_moving=false;
  unsigned long turn_time = abs(degrees) / turn_speed;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Right wheel forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // Left wheel backward
  analogWrite(ENA, 180* trim); analogWrite(ENB, 180);
  theta += degrees; // Update our angle
  delay(turn_time); 
  brake();
  
}

void brake() {
  update_position();
  is_moving=false;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  delay(100);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void update_position() {
  unsigned long now = millis();
  unsigned long dt = now - last_time;
  last_time = now;

  if (is_moving) {
    float dist_traveled = move_speed * dt;
    float rad = theta * 0.01745; 
    x += dist_traveled * cos(rad);
    y += dist_traveled * sin(rad);
  }
}

void send_n_receive(String data) {
  if (WiFi.status() == WL_CONNECTED) {
    
    http.begin(client, server+"/data"); // It uses the global ones you already made
    http.addHeader("Content-Type", "application/json");
    
    int http_code = http.POST(data);
    
    if (http_code > 0) {
      String payload = http.getString();
      
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        // Extract the target coordinates sent by Flask
        target_x = doc["tx"].as<int>();
        target_y = doc["ty"].as<int>();
        http.end();
      } 
      else{
        http.end();
        display("[LOG] JSON Parse Error: " + String(error.f_str()));
      }
    }
    else{
      http.end();
    }
  }
}

void display(const String &msg){
  if (WiFi.status() != WL_CONNECTED) return;

  display_http.begin(client, server + "/display");
  display_http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
  doc["msg"] = msg;
  String body;
  serializeJson(doc, body);

  display_http.POST(body);
  display_http.end();
}

void reconnect(){
  if (WiFi.status() != WL_CONNECTED){
  WiFi.begin(ssid, password);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  int retry_count = 0;
  while (WiFi.status() != WL_CONNECTED && retry_count < 10) {
    delay(100);
    retry_count++;
  }
  }
}


void scan_and_send() {
  // Use a larger buffer for 3D data points
  DynamicJsonDocument doc(8192); 
  JsonArray data = doc.to<JsonArray>();

  // Loop through Tilt angles (Vertical)
  for (int t = 0; t < 3; t++) {
    int current_tilt = tilt_angles[t];
    tilt.write(current_tilt);
    delay(200); // Wait for tilt servo

    // Loop through Pan angles (Horizontal)
    for (int p = scan_start_pan; p <= scan_end_pan; p += pan_step) {
      pan.write(p);
      delay(150); // Wait for pan servo

      unsigned int distance = sonar.ping_cm();
      if (distance == 0) distance = max_distance; 

      // Create a data point object
      JsonObject point = data.createNestedObject();
      point["x"] = x; 
      point["y"] = y;
      point["Q"] = theta;
      point["dist"] = distance;
      point["pan"] = p;
      point["tilt"] = current_tilt;
    }
  }

  // Send the 3D data packet
  String payload;
  serializeJson(doc, payload);
  send_n_receive(payload);
}