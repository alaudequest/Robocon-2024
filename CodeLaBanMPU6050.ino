#include "I2Cdev.h"
#include "string.h"
#include "SimpleKalmanFilter.h"
#include "stdio.h"
SimpleKalmanFilter loc_goc (1, 1, 0.1);

#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN PB8  
#define LED_PIN PB13 
#define LED_OK PB14 
#define DataPin PB1
#define ReadyPin PB0
#define UserBtn PB5

bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;                   
VectorFloat gravity;    

float ypr[3],yprPre[3];  
int goc, gochc, gocloc, gocraw;
int xoay, gocxoay, bd, i;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}
unsigned long long time1, timeled, timedata;
int offset, DaGetOffSet;
float xdeg,ydeg,zdeg;

String s;
void plotterAddParam(String *outputStr, char* name, float data){
  *outputStr+=' ';
  *outputStr+=name;
  *outputStr+=':';
  *outputStr+= String(data);
  *outputStr+= ",\t";
}

void plotterPrint(String *outputStr){
  *outputStr+= '\n';
  Serial.print(*outputStr);
  *outputStr = "";
}



void setup() {

    Serial.begin(115200);
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(PB5, INPUT_PULLUP);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_OK, OUTPUT);

    pinMode(DataPin, OUTPUT);
    pinMode(ReadyPin, OUTPUT);
    digitalWrite(ReadyPin, LOW);
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial);    
    Serial.println("Initialize MPU with DMP firmware loading");
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if(devStatus)
      Serial.println("Fail to load firmware to DMP");
    else 
      Serial.println("Load firmware to DMP succesfully, calibrating accel and gyro ...");
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    Serial.println("MPU6050 Ready");
    bd = gocxoay;
    xoay = 0;
    
    delay(500);
    digitalWrite(LED_PIN, HIGH);    
}

void getDataDMP(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer); 
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      xdeg = ypr[0] * 180.0/PI;
      ydeg = ypr[1] * 180.0/PI;
      zdeg = ypr[2] * 180.0/PI;

      plotterAddParam(&s,"x",xdeg);
      plotterAddParam(&s,"y",ydeg);
      plotterAddParam(&s,"z",zdeg);
      plotterPrint(&s);
      // goc = ypr[0] * 180/PI; 
      // gocf = (1-0.96)*gocf_p+0.96*goc;

      // gocf_p = gocf;

      // gocxoay = gocf;
      
      // if(gocxoay < 0) gocxoay = 360 + gocxoay;
      // if(gocxoay != bd){
      //     xoay += kt(bd, gocxoay);;
      //     bd = gocxoay;
      // }
  }
}

void loop() {
  if(mpuInterrupt){
    getDataDMP();
    mpuInterrupt = false;
  }
  if(!digitalRead(UserBtn)){
    Serial.println("press btn");
    setZero();
  }
  if(millis() - time1 > 100){
      blinkState = !blinkState;
      digitalWrite(LED_OK, blinkState);
      time1 = millis();  
  }
}

void setZero(){
  mpu.setXGyroOffset(-100);
  mpu.setYGyroOffset(-100);
  mpu.setZGyroOffset(-100);
}
