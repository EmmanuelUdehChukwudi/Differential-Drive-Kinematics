#include "PinChangeInterrupt.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

#define Encoder_R_output_B 7
#define Encoder_L_output_B 8

#define Encoder_R_output_A 3 
#define Encoder_L_output_A 2 

#define R_speed_pin 6
#define R_dir_1_pin 9
#define R_dir_2_pin 10

#define L_speed_pin 5
#define L_dir_1_pin 11
#define L_dir_2_pin 12

int right_count = 0;
int left_count = 0;

float wheel_diameter = 8.40; // centimeters
float L_encoder_CPR = 207;
float R_encoder_CPR = 209;

int base_speed = 90; 

// PID parameters
float error;
float set_point = 0.0;
double integral_term = 0;
double derivative_term = 0;
double proportional_term = 0;
double previous_error = 0;
double output;

float Kp = 350;
float Ki = 100;
float Kd = 1.5;

double sample_time = 10; // Sample time in milliseconds
unsigned long last_time = 0;

void setup() {
  Serial.begin(9600); 
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  pinMode(Encoder_R_output_A, INPUT); 
  pinMode(Encoder_L_output_A, INPUT); 
  pinMode(Encoder_R_output_B, INPUT_PULLUP);
  pinMode(Encoder_L_output_B, INPUT_PULLUP);

  pinMode(R_speed_pin, OUTPUT);
  pinMode(R_dir_1_pin, OUTPUT);
  pinMode(R_dir_2_pin, OUTPUT);

  pinMode(L_speed_pin, OUTPUT);
  pinMode(L_dir_1_pin, OUTPUT);
  pinMode(L_dir_2_pin, OUTPUT);

  attachPCINT(digitalPinToPCINT(Encoder_R_output_B), do_right_motor, CHANGE);
  attachPCINT(digitalPinToPCINT(Encoder_L_output_B), do_left_motor, CHANGE);

  attachInterrupt(digitalPinToInterrupt(Encoder_R_output_A), do_right_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_L_output_A), do_left_motor, RISING);
  delay(5000);
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_time >= sample_time) {
    last_time = current_time;
    float res = ReadGyro();
    float output = PID_Control(set_point, res);

    int left_speed = base_speed - output;
    int right_speed = base_speed + output;
    left_speed = constrain(left_speed, 0, 200);
    right_speed = constrain(right_speed, 0, 200);

    // Move motors
    MoveMotor_L(left_speed, 1); 
    MoveMotor_R(right_speed, 1); 
    Serial.print(left_speed);
    Serial.print(",");
    Serial.println(right_speed);

    // Send wheel speeds to ESP32
    String wheelSpeeds = String(left_speed) + "," + String(right_speed);
    Serial.println(wheelSpeeds);
  }

  // Check if data is available from ESP32
  // if (Serial.available()) {
  //   set_point = Serial.parseFloat();
  //   // Serial.print("New setpoint received: ");
  //   // Serial.println(set_point);
  // }
}

void do_right_motor() {
  int b = digitalRead(Encoder_R_output_B);
  if (b > 0) {
    right_count--;
  } else {
    right_count++;
  }
}

void do_left_motor() {
  int b = digitalRead(Encoder_L_output_B);
  if (b > 0) {
    left_count++;
  } else {
    left_count--;
  }
}

float ReadGyro() {
  mpu6050.update();
  float angle_z = mpu6050.getAngleZ();
  return angle_z;
}

void MoveMotor_R(int speed, int dir) {
  if (dir == 1) { // forward
    digitalWrite(R_dir_1_pin, HIGH);
    digitalWrite(R_dir_2_pin, LOW);
  } else if(dir == -1) { // reverse
    digitalWrite(R_dir_1_pin, LOW);
    digitalWrite(R_dir_2_pin, HIGH);
  } else if(dir == 0) {
    digitalWrite(R_dir_1_pin, LOW);
    digitalWrite(R_dir_2_pin, LOW);
    speed = 0;
  }
  analogWrite(R_speed_pin, speed);
}

void MoveMotor_L(int speed, int dir) {
  if (dir == 1) { // forward
    digitalWrite(L_dir_1_pin, HIGH);
    digitalWrite(L_dir_2_pin, LOW);
  } else if (dir == -1) { // reverse
    digitalWrite(L_dir_1_pin, LOW);
    digitalWrite(L_dir_2_pin, HIGH);
  } else if (dir == 0) {
    digitalWrite(L_dir_1_pin, LOW);
    digitalWrite(L_dir_2_pin, LOW);
    speed = 0;
  }
  analogWrite(L_speed_pin, speed);
}

double PID_Control(float set_point, float feedback) {
  error = set_point - feedback;
  proportional_term = Kp * error;
  integral_term += Ki * error * sample_time / 1000.0; 
  derivative_term = Kd * (error - previous_error) / (sample_time / 1000.0);
  output = proportional_term + integral_term + derivative_term;
  previous_error = error;
  return output;
}

float CalculateDistance(float count, float cpr) {
  float distance = (count * 2 * PI * (wheel_diameter / 2)) / cpr;
  return distance;
}
