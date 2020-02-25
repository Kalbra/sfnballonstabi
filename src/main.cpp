#include <Arduino.h>              //Arduino Framework for basic commands
#include <PID_v1.h>               //PID libarry 
#include <Wire.h>                 //I2C Bus libarry
#include <Servo.h>                //Servo libarry(PWM Signal)
#include "MPU6050_tockn.h"        //Libarry MPU6050 reading via I2C(Wire)
#include "HMC5883L.h"

#define MPU6050_ADRESS     0x68    //Adress for MPPU6050

bool minusInput;       

double Setpointx = 0;               //Point for PID
double Inputx    = 0;               //Value from MPU6050 for PID
double Outputx   = 0;               //Output for Servo from PID

double Setpointy = 0;               //Point for PID
double Inputy    = 0;               //Value from MPU6050 for PID
double Outputy   = 0;               //Output for Servo from PID

double Outputreadyforservox;        //Value mapped after PID calculation
double Outputreadyforservoy;        //Value mapped after PID calculation

double Kpx=100, Kix=100, Kdx=0;    //PID Values 
double Kpy=100, Kiy=100, Kdy=0;    //PID Values 

PID PIDx(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, DIRECT); //Define PID
PID PIDy(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, DIRECT);

MPU6050 mpu6050(Wire);             //Define MPU6050

Servo tester;                      //Define Servos 

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  
  tester.attach(3);                //Define servo port 

  servo1.attach(11);
  servo2.attach(8);
  servo3.attach(5);
  servo4.attach(2);

  Serial.begin(9600);              //Begin serial connection at 9600 Baud

  Wire.begin();                    //Begin I2C bus

  mpu6050.begin();                 //Begin MPU6050 reading 

  mpu6050.calcGyroOffsets(true);   //Config MPU6050

  PIDx.SetMode(AUTOMATIC);        //Turn the PID on
  PIDy.SetMode(AUTOMATIC); 

  PIDx.SetTunings(Kpx, Kix, Kdx);    //Adjust PID values
  PIDy.SetTunings(Kpy, Kiy, Kdy);
}

void loop() {

  mpu6050.update();               //Update ther MPU6050
  
  //Make all ready for the servo
  Inputx = mpu6050.getAccAngleX(); 
  Inputy  = mpu6050.getAccAngleY();

  /*
  if(Input > 0){
    Input = -(Input);
    minusInput = false;
  }
  else{
    minusInput = true; 
  }
  */


  PIDx.Compute();                //Compute the PID 
  PIDy.Compute();

  Outputreadyforservox = map(Outputx, 0, 255, 0, 35);
  Outputreadyforservoy = map(Outputy, 0, 255, 0, 35);
  
  /*
  if(!minusInput){
    Outputreadyforservo = -(Outputreadyforservo);
  }*/

  tester.write(Outputreadyforservox + 90);  //write var to servo

       
  
  //Print the infos to serial
  Serial.print("Inputx: ");
  Serial.print(Inputx);
  Serial.print("  Inputy: ");
  Serial.println(Inputy);

  Serial.print("Outputx: ");
  Serial.print(Outputreadyforservox);
  Serial.print("  Outputy: ");
  Serial.println(Outputreadyforservoy);

  Serial.print("Setpointx: ");  
  Serial.print(Setpointx);
  Serial.print("  Setpointy: ");  
  Serial.println(Setpointy);
  Serial.println("========================================");
  delay(1000);

}