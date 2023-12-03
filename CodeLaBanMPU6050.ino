#include "I2Cdev.h"
#include "string.h"
#include "SimpleKalmanFilter.h"
#include "stdio.h"
#include "math.h"
SimpleKalmanFilter loc_goc(1, 1, 0.1);

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
#define USER_BTN PB5

#define DIFF_HIGH 0.10
#define DIFF_LOW 0.02
#define STABLE_TIME_MS 50
#define RATE_CHANGE_TIME_MS 4

bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;

typedef enum DataType {
  UNSIGN,
  SIGN,
  FLOAT,
} DataType;

float ypr[3], yprPre[3];
int goc, gochc, gocloc, gocraw;
int xoay, gocxoay, bd, i;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned long long time1, timeled, timedata;
int offset, DaGetOffSet;

typedef struct AxisDegree {
  float xdeg;
  float ydeg;
  float zdeg;
} AxisDegree;

AxisDegree axCurrent, axPre, axSteady;
uint64_t tPre, tDiff, tStable;
uint8_t lockStableValue = false;

String s;

template <class val>
void plotterAddParam(String *outputStr, char *name, val data) {
  *outputStr += name;
  *outputStr += ':';
  *outputStr += String(data);
  *outputStr += "\t";
}


void plotterAddParam2(String *outputStr, char *name, void *data, DataType type) {
  uint32_t *a = (uint32_t *)data;
  int32_t *b = (int32_t *)data;
  float *c = (float *)data;
  *outputStr += name;
  *outputStr += ':';
  switch (type) {
    case UNSIGN:
      *outputStr += String(*a);
      break;
    case SIGN:
      *outputStr += String(*b);
      break;
    case FLOAT:
      *outputStr += String(*c);
      break;
  }
  *outputStr += "\t";
}

void plotterPrint(String *outputStr) {
  *outputStr += '\n';
  Serial.print(*outputStr);
  *outputStr = "";
}



void setup() 
{

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

  while (!Serial)
    ;
  Serial.println("Initialize MPU with DMP firmware loading");
  mpu.initialize();
  // mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  // mpu.setDLPFMode(MPU6050_DLPF_BW_5);
  // mpu.setDHPFMode(MPU6050_DHPF_5);
  // mpu.setXFineGain(-9);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus)
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


void getDataDMP() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    axCurrent.xdeg = ypr[1] * 180.0 / PI;
    axCurrent.ydeg = ypr[2] * 180.0 / PI;
    axCurrent.zdeg = ypr[0] * 180.0 / PI;

    AxisDegree axDiff;
    tDiff = millis() - tPre;
    axDiff.xdeg = abs(axCurrent.xdeg - axPre.xdeg);
    axDiff.ydeg = abs(axCurrent.ydeg - axPre.ydeg);
    axDiff.zdeg = abs(axCurrent.zdeg - axPre.zdeg);

    if (axDiff.xdeg <= DIFF_LOW
        && axDiff.ydeg <= DIFF_LOW
        && axDiff.zdeg <= DIFF_LOW
        && tStable < STABLE_TIME_MS) {
      tStable += tDiff;
    } else if (tStable >= STABLE_TIME_MS && !lockStableValue) {
      lockStableValue = true;
      axSteady.xdeg = axCurrent.xdeg;
      axSteady.ydeg = axCurrent.ydeg;
      axSteady.zdeg = axCurrent.zdeg;
    }

    // update if axis rate change is bigger a threshold in short period of time, and reset stable time
    if ((axDiff.xdeg > DIFF_HIGH
         || axDiff.ydeg > DIFF_HIGH
         || axDiff.zdeg > DIFF_HIGH)) {
      tStable = 0;
      lockStableValue = false;
    }
    tPre = millis();
    axPre.xdeg = axCurrent.xdeg;
    axPre.ydeg = axCurrent.ydeg;
    axPre.zdeg = axCurrent.zdeg;

    plotterAddParam<float>(&s,"x",axCurrent.xdeg);
    plotterAddParam<float>(&s,"y",axCurrent.ydeg);
    plotterAddParam<float>(&s,"z",axCurrent.zdeg);
    plotterAddParam<float>(&s,"sx",axSteady.xdeg);
    plotterAddParam<float>(&s,"sy",axSteady.ydeg);
    plotterAddParam<float>(&s,"sz",axSteady.zdeg);
    plotterAddParam<uint32_t>(&s,"tStable",tStable);

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
  if (mpuInterrupt) {
    getDataDMP();
    mpuInterrupt = false;
  }
  if (!digitalRead(USER_BTN)) {
    Serial.println("press btn");
    setZero();
  }
  if (millis() - time1 > 100) {
    blinkState = !blinkState;
    digitalWrite(LED_OK, blinkState);
    time1 = millis();
  }
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    if (!s.compareTo("zero")) {
      setZero();
    }
  }
}

void setZero() {
  mpu.setDMPEnabled(false);
  if (mpu.dmpInitialize()) return;
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(185);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
}
