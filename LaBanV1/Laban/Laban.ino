#include "I2Cdev.h"

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter loc_goc (1, 1, 0.01);

#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN PB8  
#define LED_PIN PB13 

#define LED_OK PB14 

#define DataPin PB1

#define ReadyPin PB0

bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     

uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;                   
VectorFloat gravity;    

float ypr[3];  
int goc, gochc, gocloc, gocraw;
int xoay, gocxoay, bd, i;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}
unsigned long long time1, timeled;
int offset, DaGetOffSet;

uint16_t dl;
uint8_t Data[5];

void setup() {

    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(PB5, INPUT);
    
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

    Serial.begin(115200);
    while (!Serial);    
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    bd = gocxoay;
    xoay = 0;
    
    delay(500);
    digitalWrite(LED_PIN, HIGH);    
}

void loop() {
    doc();
    
    if(!digitalRead(PB5)){
        xoay = 0;
        delay(100);
        while(1) if(digitalRead(PB5)){
            delay(100);
            break;       
        }
    }

    if(millis() - timeled > 5){
        blinkState = !blinkState;
        digitalWrite(LED_OK, blinkState);
        timeled = millis();  
        
        Data[0] = 0xAA; 
        if(xoay >= 0){
            Data[1] = 1;
        }
        else if(xoay < 0){
            Data[1] = 0;
        }
        dl = abs(xoay);
        Data[2] = dl >> 8;
        Data[3] = dl;
        Data[4] = 10;
        Serial.write(Data, 5);
    }
    
//    if(millis() - timeled > 100){
//        blinkState = !blinkState;
//        digitalWrite(LED_OK, blinkState);
//        timeled = millis();  
//        
//        dl = abs(xoay);
//        if(xoay >= 0){
//            Serial.print("+");
//        }
//        else if(xoay < 0){
//            Serial.print("-");
//        }
//        Serial.println(dl);
//    }


}