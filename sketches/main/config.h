#include <Servo.h>

// Motor
const int IN1 = D5; //  Motor A Forward
const int IN2 = D6; //  Motor A Backward
const int ENA = D8; //  Motor A Speed (PWM)
const int IN4 = D7; //   Motor B Backward
const int IN3 = D0; //   Motor B Forward
const int ENB = D1;  //  Motor B Speed (PWM)
const int pan_servo_pin = D3; //   Pan Motor
const int tilt_servo_pin = D9;  //  Tilt Motor



Servo pan;
Servo tilt;


void forward();
void rotate();
void brake();


void setup() {
  // Initialize all pins as OUTPUT
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(tilt_servo_pin, OUTPUT); pinMode(pan_servo_pin, OUTPUT);
  
  pan.attach(pan_servo_pin); 
  tilt.attach(tilt_servo_pin);
}   

void loop() {
  // Continuous Servo Sweep to test range and power
  delay(3000); // 3 second "Safety Start" to put the robot down
  
  forward();
}

// --- MOVEMENT FUNCTIONS ---

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 200*0.9);
  analogWrite(ENB, 200); 
  delay(10000);
  brake(); // Changed from break() to brake()
}

void rotate() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180); analogWrite(ENB, 180);
  delay(2000);
  brake(); // Changed from break() to brake()
}

void brake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  delay(100);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}