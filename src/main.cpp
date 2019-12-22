#include <Arduino.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#include <Wire.h>

#define MPU6050_ADRESS     0x68
#define ADDR               0x0d

#define Mode_Standby       0b00000000
#define Mode_Continuous    0b00000001
//Output data rate
#define ODR_10Hz           0b00000000
#define ODR_50Hz           0b00000100
#define ODR_100Hz          0b00001000
#define ODR_200Hz          0b00001100
//Measure range
#define RNG_2G             0b00000000
#define RNG_8G             0b00010000
//Over sampling rate
#define OSR_512            0b00000000
#define OSR_256            0b01000000
#define OSR_128            0b10000000
#define OSR_64             0b11000000

/************************Variables***********************/
const int ACCEL_OFFSET   = 200;
const int GYRO_OFFSET    = 151;  // 151
const int GYRO_SENSITITY = 131;  // 131 is sensivity of gyro from data sheet
const float GYRO_SCALE   = 0.2; //  0.02 by default - tweak as required
const float LOOP_TIME    = 0.15; // 0.1 = 100ms

int valuebig             = 110;
int valuesmall           = 70;   

int servo1grund          = -7;
int servo2grund          = -11;
int servo3grund          = -5;
int servo4grund          = 0;

int highshigh            = 0;
bool action              = false;

int accValue[3], accAngle[3], gyroValue[3], temperature, accCorr;
float gyroAngle[3], gyroCorr;

float azimut;
int x,y,z;
int azi;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Adafruit_BMP085 bmp;
/********************************************************/
/**************************libfunctions******************/
void writeRegister(uint8_t reg, uint8_t val){
  Wire.beginTransmission(ADDR); //start talking
  Wire.write(reg); 
  Wire.write(val);
  Wire.endTransmission();
}

void softReset() {
  writeRegister(0x0a,0x80);
  writeRegister(0x0b,0x01);
}

void setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode) {
  writeRegister(9,overSampling | range | dataRate | mode);
}

void readData(int * x, int * y, int * z) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 6);
  *x = Wire.read(); //LSB  x
  *x |= Wire.read() << 8; //MSB  x
  *y = Wire.read(); //LSB  z
  *y |= Wire.read() << 8; //MSB z
  *z = Wire.read(); //LSB y
  *z |= Wire.read() << 8; //MSB y  
}
/********************************************************/
/************************servobefehl*********************/
void left(){
  servo1.write(valuesmall + servo1grund);
  servo3.write(valuebig + servo3grund); 
  }
  
void right(){
  servo1.write(valuebig + servo1grund);
  servo3.write(valuesmall + servo3grund);
  }

void front(){
  servo2.write(valuesmall + servo2grund);
  servo4.write(valuebig + servo4grund);
  }

void back(){
  servo2.write(valuebig + servo2grund);
  servo4.write(valuesmall + servo4grund);
  }

void rotateleft() {
  servo1.write(valuebig + servo1grund);
  servo2.write(valuebig + servo2grund);
  servo3.write(valuebig + servo3grund);
  servo4.write(valuebig + servo4grund);
  }

void rotateright() {
  servo1.write(valuesmall + servo1grund);
  servo2.write(valuesmall + servo2grund);
  servo3.write(valuesmall + servo3grund);
  servo4.write(valuesmall + servo4grund);
  }

void reset(){
  servo1.write(90 + servo1grund);
  servo2.write(90 + servo2grund);
  servo3.write(90 + servo3grund);
  servo4.write(90 + servo4grund);
  }
/********************************************************/
/***************************logger***********************/
void logging(String logstr){
  Serial.println(logstr);
}
/********************************************************/

void lookfooparation(int high){
  if(highshigh > high){
    action = true;
    logging("Starting Stable");
  }
  else
  {
    highshigh = high;
    action = false;
  }
}

void setup() {
  //Begin Serial Commonication
  Serial.begin(9600);
  Serial.flush();
  /***********************************************/
logging("Setup");
  //Servo define
  servo1.attach(11);
  servo2.attach(8);
  servo3.attach(5);
  servo4.attach(2);
  /***********************************************/

  //Wire comminication start
  Wire.begin();
  softReset();
  setCtrlRegister(OSR_128,RNG_2G,ODR_100Hz,Mode_Continuous);
  Wire.beginTransmission(MPU6050_ADRESS); 
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  /***********************************************/

  //Define Pressure Sensor
  bmp.begin();
  /***********************************************/
}

void loop() {
logging("Start loop");
  Serial.print(bmp.readTemperature());
 //triple axis data
  
  azi = azimut+180; //azimut only positiv as integer
  readData(&x, &y, &z);
  azimut = -atan2(y,x) * 180.0/PI;
  Wire.beginTransmission(MPU6050_ADRESS);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU6050_ADRESS, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  for(byte i=0; i<3; i++) {
    accValue[i] = Wire.read()<<8 | Wire.read(); // reading registers: ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT
  }
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  for(byte i=0; i<3; i++) {
    gyroValue[i] = Wire.read()<<8 | Wire.read(); // reading registers: GYRO_XOUT, GYRO_>OUT, GYRO_ZOUT
  }

 // Serial.print("Angle: ");
 logging("Angle: ");
  for(byte i=0; i<3; i++) {
    accCorr = accValue[i] - ACCEL_OFFSET;
    accCorr = map(accCorr, -16800, 16800, -90, 90);
    accAngle[i] = constrain(accCorr, -90, 90);
   /* Serial.print(accAngle[i]);
    Serial.print("\t"); */
    logging(String(accAngle[i]));
  }
 // Serial.print("| Gyro: \t");

  for(byte i=0; i<3; i++) {
    gyroCorr = (float)((gyroValue[i]/GYRO_SENSITITY) - GYRO_OFFSET);
    gyroAngle[i] = (gyroCorr * GYRO_SCALE) * -LOOP_TIME;
  /*  Serial.print(gyroAngle[i]);
    Serial.print("\t");  */
    logging(String(gyroAngle[i]));

  }

  Serial.println(" ");
  Serial.print("Richtung: ");
  Serial.println(azimut);
  delay(LOOP_TIME * 1000);

  ///////////////////////looking/////////////////////////


if(action){

if (azimut > 20){
  rotateright();
  }
else if (azimut < -20){
  rotateleft();
  }
else {
  reset();

if (accAngle[1] > 7){
  right();
  }
else if (accAngle[1] < -7){
  left();
  }
else if (accAngle[0] > 7){
   back();
  }
else if (accAngle[0] < -7){
   front();
  }
else{
  reset();
  }
}
}
}