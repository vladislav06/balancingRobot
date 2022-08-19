#include "Wire.h"
#include <MPU6050_light.h>

#define m1_1 9
#define m1_2 10
#define m2_1 5
#define m2_2 6

#define m1_offset 1
#define m2_offset 1

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  pinMode(m1_1, OUTPUT);
  pinMode(m1_2, OUTPUT);
  pinMode(m2_1, OUTPUT);
  pinMode(m2_2, OUTPUT);

  Serial.begin(9600);
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  mpu.calcOffsets(); // gyro and accelero

  Serial.println("P I D out speed");

}

float Pout = 0;
float Iout = 0;
float Dout = 0;
float last_err = 0;
float _integral = 0;

int pid(float set_angle, float curr_angle, float p, float i, float d, float d_time) {
  // Calculate error
  float error =  curr_angle - set_angle ;

  // Proportional term
  Pout = p * error;

  // Integral term
  _integral += error * d_time;
  Iout = i * _integral;

  // Derivative term
  float derivative = (error - last_err) / d_time;
  Dout = d * derivative;

  //restict indicidual terms
  if (Pout > 255)
    Pout = 255;
  else if (Pout < -255)
    Pout = -255;

  if (Iout > 255)
    Iout = 255;
  else if (Iout < -255)
    Iout = -255;

  if (Dout > 255)
    Dout = 255;
  else if (Dout < -255)
    Dout = -255;

  //restirct internal values
  if (_integral > 255)
    _integral = 255;
  else if (_integral < -255)
    _integral = -255;


  // Calculate total output
  int output = Pout + Iout + Dout;


  // Restrict to max/min
  if ( output > 255 )
    output = 255;
  else if ( output < -255)
    output = -255;

  // Save error to previous error
  last_err = error;

  return output;
}


// 0 - 255 forward movment
// 0 - -255 backward movement
// -255  -  255 
// 
void move(int dir) {
  if (dir > 0) {
    digitalWrite(m1_2, LOW);
    digitalWrite(m2_2, LOW);

    //use motor offests cuz each motor is slightly different
    int m1 = dir * m1_offset;
    int m2 = dir * m2_offset;

    if (m1 > 255) {
      m1 = 255;
    }
    if (m2 > 255) {
      m2 = 255;
    }

    analogWrite(m1_1, m1);
    analogWrite(m2_1, m2);
  } else {
    dir = -dir;
    digitalWrite(m1_1, LOW);
    digitalWrite(m2_1, LOW);

    int m1 = dir * m1_offset;
    int m2 = dir * m2_offset;

    if (m1 > 255) {
      m1 = 255;
    }
    if (m2 > 255) {
      m2 = 255;
    }

    analogWrite(m1_2, m1);
    analogWrite(m2_2, m2);
  }

}
int del_time = 0;
void debug(int dir) {
  Serial.println(del_time);
}


void loop() {
  mpu.update();
  del_time = millis() - timer;

  //        set_angle, curr_angle,       p,   i,   d,   d_time
  int dir = pid(0,     mpu.getAngleY(),  40, 0.1, 50, del_time);
  timer = millis();
  move(dir);

  debug(dir);

  
}
