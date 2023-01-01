// Basic demo for accelerometer readings from Adafruit MPU6050

#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;
// orientation/motion vars
Quaternion curRotation;             // [w, x, y, z]         quaternion container
Quaternion prevRotation;            // [w, x, y, z]         quaternion container
static Quaternion prevOrientation;  // [w, x, y, z]         quaternion container
static Quaternion curOrientation;   // [w, x, y, z]         quaternion container
VectorInt16 curAccel;
VectorInt16 prevAccel;
VectorInt16 curDeltAccel;
VectorInt16 prevDeltAccel;

#define CLASH_THRESHOLD 10
volatile bool mpuInterruptDetected = false;  // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;                        // holds actual interrupt status byte from MPU
uint16_t DMPpacketSize;                      // expected DMP packet size (default is 42 bytes)
uint8_t mpuFifoBuffer[64];                   // FIFO storage buffer
uint16_t mpuFifoCount;                       // count of all bytes currently in FIFO
#define MPU_INTERRUPT_PIN 2
bool dmpReady = false;
I2Cdev i2ccomm;



inline void dmpDataReady() {
  mpuInterruptDetected = true;
}  //dmpDataReady

void ISR_MPUInterrupt() {
  Serial.println("CLASH");
  if (false) {  // TODO SABER CLASH HERE
    // FX_Clash();
  }
}
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  mpu.initialize();
  Serial.println(
    mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize_light();

  mpu.setXAccelOffset(-3289);
  mpu.setYAccelOffset(-1392);
  mpu.setZAccelOffset(516);
  mpu.setXGyroOffset(240);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(27);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    DMPpacketSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP READY"));
  }

  // configure the motion interrupt for clash recognition
  // INT_PIN_CFG register
  mpu.setDLPFMode(3);  // Digital Low-Pass Frequenct
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
  i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x37, 0xF0);
  // enable only Motion Interrupt
  i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x38, 0x40);
  mpuIntStatus = mpu.getIntStatus();

  pinMode(MPU_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(0, ISR_MPUInterrupt, FALLING);  // ISR
  Serial.println(F("IMU intterupts READY"));
}

void loop() {
  motionEngine();

  if (abs(curRotation.w * 1000) < 999   // some rotation movement have been initiated
      and ((abs(curDeltAccel.x) > 1000  // and it has suffisent power on a certain axis
            or abs(curDeltAccel.z) > 1000
            or abs(curDeltAccel.y) > 1000 * 10))) {
    prevDeltAccel = curDeltAccel;

    // Serial.print(F("SWING\ttime="));
    // Serial.println(millis() - sndSuppress);
    Serial.print(F("\t\tcurRotation\tw="));

    Serial.print(curRotation.w * 1000);
    Serial.print(F("\t\tx= "));
    Serial.print(curRotation.x);
    Serial.print(F("\t\ty= "));
    Serial.print(curRotation.y);
    Serial.print(F("\t\tz= "));
    Serial.print(curRotation.z);
    Serial.print(F("\t\tAcceleration\tx= "));
    Serial.print(curDeltAccel.x);
    Serial.print(F("\ty= "));
    Serial.print(curDeltAccel.y);
    Serial.print(F("\tz= "));
    Serial.println(curDeltAccel.z);
  }
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
}  //printQuaternion

inline void motionEngine() {
  if (!dmpReady)
    return;

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

    //We calculate the rotation quaternion since last orientation
    prevRotation = curRotation;
    curRotation = prevOrientation.getProduct(
      curOrientation.getNormalized());
    // display quaternion values in easy matrix form: w x y z
    printQuaternion(curRotation);
  }
}