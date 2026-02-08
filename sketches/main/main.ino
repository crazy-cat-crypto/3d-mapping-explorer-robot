#include <NewPing.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>


// --- WIFI SETTINGS ---
const char* ssid = "oldtaj_2.5";
const char* password = "9821976791@sad";
String server = "http://192.168.1.213:5000";

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
  pan.attach(pan_servo_pin); tilt.attach(tilt_servo_pin);

  


  WiFi.begin(ssid, password);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  display("Wifi connected");
  last_time = millis();
  forward(150);
  delay(1000);
  brake();
}  

void loop() {
  delay(5000);
  display("Wifi connected");
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

void rotate(int degrees) {
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