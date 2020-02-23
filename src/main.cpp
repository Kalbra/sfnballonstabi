#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Servo.h>
#include "/home/kalle/Documents/PlatformIO/Projects/pidgonelsteuerung/lib/MPU6050_tockn.h"

#define MPU6050_ADRESS     0x68


double serialinput = 0;

double Setpoint ; 
double Input; 
double Output ;

double Outputreadyforservo;

double Kp=100, Ki=100, Kd=0; 

PID myPID(&serialinput, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu6050(Wire);

Servo tester;

void setup() {
  tester.attach(2);

  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Setpoint = 0;
  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetTunings(Kp, Ki, Kd);

}

void loop() {

  mpu6050.update();
  
  serialinput = mpu6050.getAccAngleX();

  Outputreadyforservo = map(Output, 0, 255, 0, 35);

  tester.write(Outputreadyforservo+ 90);

  myPID.Compute();
  
  Serial.print("Input: ");
  Serial.println(serialinput);
  Serial.print("Output: ");
  Serial.println(Outputreadyforservo);
  Serial.print("Setpoint: ");  
  Serial.println(Setpoint);
  Serial.println("=============================");
  delay(1000);

}