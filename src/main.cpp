#include <Arduino.h>              //Arduino Framework for basic commands
#include <PID_v1.h>               //PID libarry 
#include <Wire.h>                 //I2C Bus libarry
#include <Servo.h>                //Servo libarry(PWM Signal)
#include "MPU6050_tockn.h"        //Libarry MPU6050 reading via I2C(Wire)

#define MPU6050_ADRESS     0x68    //Adress for MPPU6050
       

double Setpoint = 0;               //Point for PID
double Input    = 0;               //Value from MPU6050 for PID
double Output   = 0;               //Output for Servo from PID

double Outputreadyforservo;        //Value mapped after PID calculation

double Kp=100, Ki=100, Kd=0;       //PID Values 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //Define PID

MPU6050 mpu6050(Wire);             //Define MPU6050

Servo tester;                      //Define Servos 


void setup() {
  
  tester.attach(2);                //Define servo port 

  Serial.begin(9600);              //Begin serial connection at 9600 Baud

  Wire.begin();                    //Begin I2C bus

  mpu6050.begin();                 //Begin MPU6050 reading 

  mpu6050.calcGyroOffsets(true);   //Config MPU6050

  myPID.SetMode(AUTOMATIC);        //Turn the PID on

  myPID.SetTunings(Kp, Ki, Kd);    //Adjust PID values

}

void loop() {

  mpu6050.update();               //Update ther MPU6050
  
  //Make all ready for the servo
  Input = mpu6050.getAccAngleX(); 

  myPID.Compute();                //Compute the PID

  if(Input > 0){
    double minuout = -(Output);
    Serial.println(minuout);
    Outputreadyforservo = (map(minuout, 0, 255, 0, 35));
  }
  else{
    Outputreadyforservo = -(map((Output), 0, 255, 0, 35));
  }
  //Outputreadyforservo = map(Output, 0, 255, 0, 35);

  tester.write(Outputreadyforservo + 90);  //write var to servo

                
  
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