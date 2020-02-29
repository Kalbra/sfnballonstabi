#include <Arduino.h>              //Arduino Framework for basic commands
#include <PID_v1.h>               //PID libarry 
#include <Wire.h>                 //I2C Bus libarry
#include <Servo.h>                //Servo libarry(PWM Signal)
#include "MPU6050_tockn.h"        //Libarry MPU6050 reading via I2C(Wire)
#include <includes/QMC5883L.h>
#include "calculateengine.h"

//true = left false = right

#define MPU6050_ADRESS     0x68    //Adress for MPPU6050

int compassdirection;  
int compassdirectionbefor;     

double Setpointx = 0;               //Point for PID
double Inputx    = 0;               //Value from MPU6050 for PID
double Outputx   = 0;               //Output for Servo from PID

double Setpointy = 0;               //Point for PID
double Inputy    = 0;               //Value from MPU6050 for PID
double Outputy   = 0;               //Output for Servo from PID

double Setpointcompass = 0;               //Point for PID
double Inputcompass    = 0;               //Value from MPU6050 for PID
double Outputcompass   = 0;               //Output for Servo from PID

double Outputreadyforservox;        //Value mapped after PID calculation
double Outputreadyforservoy;        //Value mapped after PID calculation

double Kpx=100, Kix=100, Kdx=0;    //PID Values 
double Kpy=100, Kiy=100, Kdy=0;    //PID Values 
double Kpcompass=100, Kicompass=100, Kdcompass=0;

int s1;
int s2;
int s3;
int s4;

pidvar xpidvar;
pidvar ypidvar;

pidvar compasspidvar;

calculate calcer;

PID PIDx(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, DIRECT); //Define PID
PID PIDy(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, DIRECT);
PID PIDcompass(&Inputcompass, &Outputcompass, &Setpointcompass, Kpcompass, Kicompass, Kdcompass, DIRECT);

MPU6050 mpu6050(Wire);             //Define MPU6050

QMC5883L compass;

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

  compass.init();
	compass.setSamplingRate(0);

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

  Inputcompass = compass.readHeading()-180;

  if(Inputcompass > 0){
      compasspidvar.direction = true;
      Inputcompass = -(Inputcompass);
  }
  else
  {
      compasspidvar.direction = false;
  }


  if(Inputx > 0){
      xpidvar.direction = true;
      Inputx = -(Inputx);
  }
  else
  {
      xpidvar.direction = false;
  }

  if(Inputy > 0){
    ypidvar.direction = true;
    Inputy = -(Inputy);
  } 
  else
  {
    ypidvar.direction = false;
  }
  
  PIDx.Compute();                //Compute the PID 
  PIDy.Compute();
  
  xpidvar.value = Outputx;
  ypidvar.value = Outputy;

  calcer.run(&s1, &s2, &s3, &s4, xpidvar, ypidvar, compasspidvar); 


  compassdirection = compass.readHeading();

  PIDx.Compute();                //Compute the PID 
  PIDy.Compute();

  Outputreadyforservox = map(Outputx, 0, 255, 0, 35);
  Outputreadyforservoy = map(Outputy, 0, 255, 0, 35);

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
  
  Serial.print("Compass: ");
  Serial.println(compassdirection);
  Serial.println("========================================");


  Serial.println(s1);
  Serial.println(s2);
  Serial.println(s3);
  Serial.println(s4);
  
  delay(1000);

}
/*
int s1;
int s2;
int s3;
int s4;

void setup(){
  Serial.begin(9600);

}
void loop(){
  pidvar x;
  x.value = 23;
  x.direction = true;

  pidvar z;
  z.value = 23;
  z.direction = true;

  calculate calc;
  
  calc.run(&s1, &s2, &s3, &s4, x, z); 

  Serial.println(s1);
  Serial.println(s2);
  Serial.println(s3);
  Serial.println(s4);
}*/