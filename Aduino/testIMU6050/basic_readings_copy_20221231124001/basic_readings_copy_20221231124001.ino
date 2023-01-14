#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
// #endif

#include "audio.h"
#include "Arduino.h"

MPU6050 mpu;

// orientation/motion vars
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

Quaternion q;                       // [w, x, y, z]         quaternion container
Quaternion curRotation;             // [w, x, y, z]         quaternion container
Quaternion prevRotation;            // [w, x, y, z]         quaternion container
static Quaternion prevOrientation;  // [w, x, y, z]         quaternion container
static Quaternion curOrientation;   // [w, x, y, z]         quaternion container
VectorInt16 curAccel;
VectorInt16 prevAccel;
VectorInt16 curDeltAccel;
VectorInt16 prevDeltAccel;
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
VectorInt16 curGyro;


// MPU control/status vars
#define CLASH_THRESHOLD 10
volatile bool mpuInterruptDetected = false;  // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;                        // holds actual interrupt status byte from MPU
uint16_t DMPpacketSize;                      // expected DMP packet size (default is 42 bytes)
uint8_t mpuFifoBuffer[64];                   // FIFO storage buffer
uint16_t mpuFifoCount;                       // count of all bytes currently in FIFO
#define MPU_INTERRUPT_PIN 6
bool dmpReady = false;
I2Cdev i2ccomm;
uint8_t fifoBuffer[64];  // FIFO storage buffer

void ISR_MPUInterrupt() {
  Serial.println("CLASH");
  if (false) {  // TODO SABER CLASH HERE
    // FX_Clash();
  }
}
inline void dmpDataReady() {
  mpuInterruptDetected = true;
}  //dmpDataReady




void setupIMU() {
  Serial.println("Setting up IMU");
  // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  // Wire.begin();
  //  Serial.println("Done init4");
  // Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  // Serial.println("Done init3");
  // Wire.setTimeOut(3000);
  // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  //   Fastwire::setup(400, true);
  // #endif


  Serial.println("Done init2");
  mpu.initialize();
  Serial.println("Done init");
  Serial.println(
    mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  // These values are hard coded for the IMU
  mpu.setXAccelOffset(-3289);
  mpu.setYAccelOffset(-1392);
  mpu.setZAccelOffset(516);
  mpu.setXGyroOffset(240);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(27);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    DMPpacketSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP READY"));
  } else {
    Serial.println(F("DMP failed to initialize"));
  }

  Serial.println(F("Setting up IMU intterupts"));
  // configure the motion interrupt for clash recognition
  // INT_PIN_CFG register
  mpu.setDLPFMode(3);  // Digital Low-Pass Frequency
  mpu.setDHPFMode(0);  // Digital High-Pass Frequency
  //mpu.setFullScaleAccelRange(3);
  mpu.setIntMotionEnabled(true);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntFIFOBufferOverflowEnabled(false);
  mpu.setIntI2CMasterEnabled(false);
  mpu.setIntDataReadyEnabled(false);
  mpu.setMotionDetectionThreshold(CLASH_THRESHOLD);  // 1mg/LSB
  mpu.setMotionDetectionDuration(2);                 // number of consecutive samples above threshold to trigger int


  // configure Interrupt with:
  // int level active low
  // int driver open drain
  // interrupt latched until read out (not 50us pulse)
  i2ccomm.writeByte(0x68, 0x37, 0xF0);
  // enable only Motion Interrupt
  i2ccomm.writeByte(0x68, 0x38, 0x40);
  mpuIntStatus = mpu.getIntStatus();

  pinMode(MPU_INTERRUPT_PIN, INPUT);
  attachInterrupt(0, ISR_MPUInterrupt, FALLING);  // ISR
  Serial.println(F("IMU intterupts READY"));
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  setupIMU();
}
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
void loop() {
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
    printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
    printf("ROLL: %3.1f \n", ypr[2] * 180 / M_PI);
  }

  // motionEngine();
}

inline void printQuaternion(Quaternion quaternion) {
  if (quaternion.w * 1000 < 999.6 || abs(quaternion.x + quaternion.y + quaternion.z) > 0.3) {
    Serial.print(F("\t\tQ\t\tw= "));
    Serial.print(quaternion.w * 1000);
    Serial.print(F("\t\tsum= "));
    Serial.println(abs(quaternion.x + quaternion.y + quaternion.z));
    // Serial.print(F("\t\tx= "));
    // Serial.print(quaternion.x);
    // Serial.print(F("\t\ty= "));
    // Serial.print(quaternion.y);
    // Serial.print(F("\t\tz= "));
    // Serial.println(quaternion.z);
  }
}

inline void motionEngine() {
  // Serial.println("Running motion engine");
  if (!dmpReady)
    return;

  // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
  //   mpu.dmpGetQuaternion(&curOrientation, fifoBuffer);
  //   Serial.print("quat\t");
  //   Serial.print(curOrientation.w);
  //   Serial.print("\t");
  //   Serial.print(curOrientation.x);
  //   Serial.print("\t");
  //   Serial.print(curOrientation.y);
  //   Serial.print("\t");
  //   Serial.println(curOrientation.z);

  //   // display initial world-frame acceleration, adjusted to remove gravity
  //   // and rotated based on known orientation from quaternion
  //   mpu.dmpGetQuaternion(&q, fifoBuffer);
  //   mpu.dmpGetAccel(&aa, fifoBuffer);
  //   mpu.dmpGetGravity(&gravity, &q);
  //   mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  //   mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  //   Serial.print("aworld\t");
  //   Serial.print(aaWorld.x);
  //   Serial.print("\t");
  //   Serial.print(aaWorld.y);
  //   Serial.print("\t");
  //   Serial.println(aaWorld.z);

  //   mpu.dmpGetGyro(&curGyro, mpuFifoBuffer);
  // }


  mpuInterruptDetected = false;
  mpuIntStatus = mpu.getIntStatus();  // INT_STATUS byte
  mpuFifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    curDeltAccel.x = 0;
    curDeltAccel.y = 0;
    curDeltAccel.z = 0;
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (mpuFifoCount < DMPpacketSize)
      mpuFifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(mpuFifoBuffer, DMPpacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    mpuFifoCount -= DMPpacketSize;

    prevOrientation = curOrientation.getConjugate();
    prevAccel = curAccel;

    mpu.dmpGetQuaternion(&curOrientation, mpuFifoBuffer);

    mpu.dmpGetAccel(&curAccel, mpuFifoBuffer);
    curDeltAccel.x = prevAccel.x - curAccel.x;
    curDeltAccel.y = prevAccel.y - curAccel.y;
    curDeltAccel.z = prevAccel.z - curAccel.z;

    mpu.dmpGetGyro(&curGyro, mpuFifoBuffer);

    //We calculate the rotation quaternion since last orientation
    prevRotation = curRotation;
    curRotation = prevOrientation.getProduct(
                    curOrientation.getNormalized());
    // display quaternion values in easy matrix form: w x y z
    // printQuaternion(curRotation);
  }
  Serial.println("Done motion engine");
}


// // i2c utility functions
// bool write_packet(byte device, int address, int data) {
//   Wire.beginTransmission(device);
//   Wire.write(address);
//   Wire.write(data);
//   Wire.endTransmission();
//   return true;
// }

// bool read_packet(byte device, byte address, byte *buffer, byte length) {
//   // send phase
//   Wire.beginTransmission(device);
//   Wire.write(address);
//   Wire.endTransmission();
//   // recieve phase
//   Wire.beginTransmission(device);
//   Wire.requestFrom(device, (uint8_t)length);
//   // how many did we get?
//   byte r = Wire.available();
//   if (r <= length) {
//     for (byte i = 0; i < r; i++) {
//       buffer[i] = Wire.read();
//       // Serial.print("."); Serial.print(buffer[i]);
//     }
//   } else {
//     // consume unexpected bytes
//     for (byte i = 0; i < length; i++) { Wire.read(); }  //  { Serial.print("!"); Serial.print(Wire.read()); }
//   }
//   Wire.endTransmission();
//   // if(r < length) { Serial.print(" packet too short! "); Serial.print(r); }
//   // if(r > length) { Serial.print(" packet too long! "); Serial.print(r); }
//   return (r == length);
// }

// /*
//  * MPU6040 Accellerometer + Gyro
//  *
//  */
// static const byte I2C_MPU6050 = 0x68;  // i2c device address
// // public properties
// void MPU6050_start() {
//   // reset
//   write_packet(I2C_MPU6050, 0x6B, 0x80);  //
//   delay(50);
//   // configuration
//   write_packet(I2C_MPU6050, 0x19, 0x01);  // Sample Rate
//   write_packet(I2C_MPU6050, 0x6B, 0x03);  // Z-Axis gyro reference used for improved stability (X or Y is fine too)
//   delay(30);
//   write_packet(I2C_MPU6050, 0x1B, 0x18);  // Gyro Configuration FS_SEL = 3
//   delay(30);
//   write_packet(I2C_MPU6050, 0x1C, 0b11101000);  // Accel Configuration AFS_SEL = 1
//   delay(20);
