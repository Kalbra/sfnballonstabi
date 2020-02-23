#include <Arduino.h>  //Arduino Framework for basic commands
#include <PID_v1.h>   //PID libarry 
#include <Wire.h>     //I2C Bus libarry
#include <Servo.h>    //Servo libarry(PWM Signal)
#include "/home/kalle/Documents/PlatformIO/Projects/pidgonelsteuerung/lib/MPU6050_tockn.h"  //Libarry MPU6050 reading via I2C(Wire)

#define MPU6050_ADRESS     0x68   //Adress for MPPU6050
       

double Setpoint = 0;  //Point for PID
double Input    = 0;  //Value from MPU6050 for PID
double Output   = 0;  //Output for Servo from PID

double Outputreadyforservo;

double Kp=100, Ki=100, Kd=0; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu6050(Wire);

Servo tester;


void setup() {
  tester.attach(2);

  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetTunings(Kp, Ki, Kd);

}

void loop() {
  //Update ther MPU6050
  mpu6050.update();
  
  //Made all ready for the servo
  Input = mpu6050.getAccAngleX();
  Outputreadyforservo = map(Output, 0, 255, 0, 35);

  //write to servo
  tester.write(Outputreadyforservo+ 90);

  //Compute the PID
  myPID.Compute();
  

  //Print the infos to serial
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Output: ");
  Serial.println(Outputreadyforservo);
  Serial.print("Setpoint: ");  
  Serial.println(Setpoint);
  Serial.println("=============================");
  delay(1000);

}