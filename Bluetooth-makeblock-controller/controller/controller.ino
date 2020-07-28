#ifndef MeArduiono_H
#define MeArduiono_H

#include <Arduino.h>
#include "MeConfig.h"

#include "MePS2.h"
MePort_Sig mePort[17] =
{
  { NC, NC }, { 10, 11 }, {  NC, NC }, { NC, NC }, {  NC, NC },
  { NC, NC }, { NC, NC }, {  NC, NC }, { NC, NC }, {  NC, NC },
  { NC, NC }, { NC, NC }, {  NC, NC }, { NC, NC }, {  NC, NC },
  { NC, NC }, { NC, NC },
};
#endif // MeArduiono_H


#include <MePS2.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

MePS2 MePS2(PORT_1); //RX, TX, inverse

#define enA 3
#define in1 4
#define in2 5
#define in3 6
#define in4 7
#define enB 8

int motorSpeedA = 0;
int motorSpeedB = 0;
int xAxis, yAxis;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  MePS2.begin(115200);
}

void loop() {
  MePS2.loop();
  Serial.println("=================");
  if (MePS2.MeAnalog(MeJOYSTICK_LX))
  {
    Serial.print("MePS2_LX value is: ");
    Serial.print(MePS2.MeAnalog(MeJOYSTICK_LX), DEC);
    Serial.print("\t");
    xAxis = MePS2.MeAnalog(MeJOYSTICK_LX);
  } else {
    xAxis = 0;
  }
  if (MePS2.MeAnalog(MeJOYSTICK_LY))
  {
    Serial.print("MePS2_LY value is: ");
    Serial.print(MePS2.MeAnalog(MeJOYSTICK_LY), DEC);
    Serial.print("\t");
    yAxis = MePS2.MeAnalog(MeJOYSTICK_LY);
  }  else {
    yAxis = 0;
  }


  if (yAxis > 120) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorSpeedA = map(abs(yAxis), 0, 255, 0, 255);
    motorSpeedB = map(abs(yAxis), 0, 255, 0, 255);
  }
  else if (yAxis < -120) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorSpeedA = map(abs(yAxis), 0, 255, 0, 255);
    motorSpeedB = map(abs(yAxis), 0, 255, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }


  if (xAxis < -120) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(abs(xAxis), 0, 255, 0, 150);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }
  if (xAxis > 120) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(abs(xAxis), 0, 255, 0, 150);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }

  
  if (motorSpeedA < 60) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 60) {
    motorSpeedB = 0;
  }
  Serial.print("SPEED A");
  Serial.print(motorSpeedA, DEC);
  Serial.print("\t");
  Serial.print("SPEED B");
  Serial.print("\t");
  Serial.println(motorSpeedB, DEC);
  Serial.println("=================");
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}
