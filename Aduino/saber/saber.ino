#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFMiniMp3.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include "NonBlockingDelay.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// ---------------------------- DEBUG -------------------------------
//  #define LS_LOOPLENGHT
#ifdef LS_LOOPLENGHT
unsigned long loopcurrenttime;
#endif


// ---------------------------- PINOUT -------------------------------
#define MP3_RX_PIN 10
#define MP3_TX_PIN 11
#define MPU_INTERRUPT_PIN 2

// ---------------------------- MP3 -------------------------------
#define GENERAL_SOUND_FOLDER 1
#define BOOT_SOUND 1
#define CHANGE_COLOR_SOUND 2
#define DRAG_SOUND 3  // When a sabers contacts any surface
#define HUM_SOUND 4
#define LOCKUP_SOUND 5  // When two sabers continiously in contact

#define RETRACT_SOUND_FOLDER 2
#define NUM_RETRACT_SOUND 2

#define IGNITE_SOUND_FOLDER 3
#define NUM_IGNITE_SOUND 5

#define CLASH_SOUND_FOLDER 4
#define NUM_CLASH_SOUND 16

#define SWING_SOUND_FOLDER 5
#define NUM_SWING_SOUND 16

class Mp3Notify;
SoftwareSerial softSerial(MP3_RX_PIN, MP3_TX_PIN);  // RXpin, TXpin
typedef DFMiniMp3<SoftwareSerial, Mp3Notify> DfMp3;
DfMp3 myDFPlayer(softSerial);

float curVolume = 0;
float goalVolume = 0;

// ---------------------------- IMU -------------------------------
MPU6050 mpu;

Quaternion curRotation;             // [w, x, y, z]         estimated of rotational forces
Quaternion prevRotation;            // [w, x, y, z]
static Quaternion prevOrientation;  // [w, x, y, z]         hold an estimate of the angle in space
static Quaternion curOrientation;   // [w, x, y, z]
VectorInt16 curAccel;
VectorInt16 prevAccel;
VectorInt16 curDeltAccel;
VectorInt16 prevDeltAccel;
VectorInt16 curGyro;
long curGyrMag, maxGyrMag, avgGyrMag = 0;
long maxGyrTimeStamp = 0;

unsigned long swing_timer;
long last_swing_time = 0;

volatile bool mpuInterruptDetected = false;
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint16_t DMPpacketSize;  // expected DMP packet size (default  42 bytes)
uint8_t mpuFifoBuffer[64];
uint16_t mpuFifoCount;  // count of all bytes currently in FIFO

bool dmpReady = false;
I2Cdev i2ccomm;

// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1
#define VOLUME 24  // From 0 to 30
#define MIN_SWING_VOLUME 20
#define MAX_SWING_VOLUME 30
#define NUM_PIXELS 144
#define ACC_SWING_THRESHOLD 1000
#define GYR_RESWING_THRESHOLD 10000
#define GYR_SLOW_SWING_THRESHOLD 9000
#define SWING_TIMEOUT_MS 500
#define RESWING_TIMEOUT_MS 400
#define VOLUME_LERP 0.043
#define CLASH_THRESHOLD 10

// ---------------------------- STATES -------------------------------
#define START_STATE 0
#define OFF_STATE 1
#define IDLE_STATE 2
#define SWING_STATE 3
#define CLASH_STATE 4

#define SWING_STATE_TIME_MS 800

class State {
private:
  char id;

public:
  State(char id) {
    this->id = id;
  }

  char getID() {
    return id;
  }

  virtual void init() {}
  virtual void run() {}
};

State* curState;
State* nextState;
bool resetSaber = false;
bool isSoundDone = false;
int prevTrack = -1;
// -------------------------------------------------------------------

class Mp3Notify {
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
    if (source & DfMp3_PlaySources_Sd) {
      Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb) {
      Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash) {
      Serial.print("Flash, ");
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode) {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }

  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track) {
    Serial.print("Play finished for #");
    Serial.println(track);

    // Ignore duplicate audio done messages
    if (track != prevTrack) {
      isSoundDone = true;
    }

    prevTrack = track;
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
    PrintlnSourceAction(source, "removed");
  }
};


inline void dmpDataReady() {
  mpuInterruptDetected = true;
}

void ISR_MPUInterrupt() {
  Serial.println("CLASH detected");
  // if (curState.getID() = '') {  // TODO SABER CLASH HERE
  //   nextState = CLASH_STATE();
  // }
}
class ClashState : public State {
public:
  ClashState()
    : State(CLASH_STATE) {}
  void init() override {
    // TODO
  }

  void run() override {
    // TODO
  }

} clashState;

static inline int8_t sign(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}

class SwingState : public State {
public:
  int swingCountDown = 8;
  long swingStateTime = 0;
  int initialSwingDirX, initialSwingDirY, initialSwingDirZ;

  SwingState()
    : State(SWING_STATE) {}

  //TODO
  bool checkDir() {
    // int diff = 0;
    // if (sign(gyrX) != initialSwingDirX) {
    //   diff++;
    // }
    // if (sign(gyrY) != initialSwingDirY) {
    //   diff++;
    // }
    // // if (sign(gyrZ) != initialSwingDirZ) {
    // //   diff++;
    // // }
    // // Serial.print("Number of axis reversed:");
    // // Serial.println(diff);

    // if (diff >= 1) {
    //   return true;
    // }

    return false;
  }

  bool hasSwingingStopped = false;
  void init() override {
    Serial.println("State: SWING");
    // myDFPlayer.setRepeatPlayCurrentTrack(false);
    // for (int i = 0; i < swingCountDown; i++) {
    //   getIMU();  // 3 ms each loop!
    // }
    // Serial.println(maxGyrMag);
    printQuaternion(curRotation);
    if (maxGyrMag < GYR_SLOW_SWING_THRESHOLD) {
      // Serial.println("SLOW SWING");
      myDFPlayer.playAdvertisement(random(1, NUM_SWING_SOUND + 1));
    } else {
      // Serial.println("FAST SWING");
      myDFPlayer.playAdvertisement(random(1000, 1002));
    }

    // initialSwingDirX = sign(gyrX);
    // initialSwingDirY = sign(gyrY);
    // initialSwingDirZ = sign(gyrZ);
    swingStateTime = millis();
    hasSwingingStopped = false;
  }
  void run() override {
    // Check is reswing in different direction
    if (checkSwing() && checkDir()) {
      Serial.println("RESWING");
      init();
    }
    // Adjust volume to saber swing speed
    // long accMag = pow(curDeltAccel.x, 2) + pow(curDeltAccel.y, 2) + pow(curDeltAccel.z,2);
    // //  long accMag = pow(curAccel.x, 2) + pow(curAccel.y, 2) + pow(curAccel.z,2);
    // uint8_t adjusted_volume = (accMag / 16666) + MIN_SWING_VOLUME; // 1666666 too large
    uint8_t adjusted_volume = (avgGyrMag / 200) + MIN_SWING_VOLUME;
    adjusted_volume = constrain(adjusted_volume, MIN_SWING_VOLUME, MAX_SWING_VOLUME);

    // Serial.print(" adjusted_volume:");
    // Serial.println(adjusted_volume);

    if (!hasSwingingStopped) {
      hasSwingingStopped = curRotation.w * 1000 < 999;
    }

    if (((millis() - swingStateTime) > SWING_STATE_TIME_MS)
       ) {
      Serial.println("RESET");
      resetSaber = true;
      isSoundDone = false;
      goalVolume = VOLUME;
      return;
    } else if (hasSwingingStopped){
      goalVolume = MIN_SWING_VOLUME;
    } else {
      goalVolume = adjusted_volume;
    }
  }

} swingState;

inline void printQuaternion(Quaternion quaternion) {
  Serial.print(F("\t\tQ\t\tw="));
  Serial.print(quaternion.w * 1000);
  Serial.print(F("\t\tx="));
  Serial.print(quaternion.x);
  Serial.print(F("\t\ty="));
  Serial.print(quaternion.y);
  Serial.print(F("\t\tz="));
  Serial.println(quaternion.z);
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

    mpu.dmpGetGyro(&curGyro, mpuFifoBuffer);
    curGyrMag = sq(curGyro.x) + sq(curGyro.y) + sq(curGyro.y);
    avgGyrMag = curGyrMag * 0.7 + avgGyrMag * (1 - 0.7);

    if (millis() - maxGyrTimeStamp > 30) {
      maxGyrMag = 0;
    }

    if (curGyrMag > maxGyrMag) {
      maxGyrMag = curGyrMag;
      maxGyrTimeStamp = millis();
    }

    // display quaternion values in easy matrix form: w x y z
    // printQuaternion(curRotation);
    // Serial.print("x");
    // Serial.print(curAccel.x);
    // Serial.print("y");
    // Serial.print(curAccel.y);
    // Serial.print("z");
    // Serial.println(curAccel.z);
  }
}

bool checkSwing() {
  motionEngine();

  if ((millis() - last_swing_time) < SWING_TIMEOUT_MS) {
    return false;
  }

  if (abs(curRotation.w * 1000) < 999  // some rotation movement have been initiated
      and ((abs(curDeltAccel.x) > ACC_SWING_THRESHOLD
            or abs(curDeltAccel.z) > ACC_SWING_THRESHOLD
            or abs(curDeltAccel.y) > ACC_SWING_THRESHOLD * 10))) {
    // Serial.print(F("Acceleration\tx="));
    // Serial.print(curDeltAccel.x);
    // Serial.print(F("\ty="));
    // Serial.print(curDeltAccel.y);
    // Serial.print(F("\tz="));
    // Serial.print(curDeltAccel.z);
    // Serial.print(F("\tcurRotation\tw="));
    // Serial.print(curRotation.w * 1000);
    // Serial.print(F("\t\tx="));
    // Serial.print(curRotation.x);
    // Serial.print(F("\t\ty="));
    // Serial.print(curRotation.y);
    // Serial.print(F("\t\tz="));
    // Serial.println(curRotation.z);


    last_swing_time = millis();
    return true;
  }

  return false;
}


class IdleState : public State {
public:
  IdleState()
    : State(IDLE_STATE) {}
  void init() override {
    resetSaber = false;

    Serial.println("State: IDLE");
    goalVolume = VOLUME;
    myDFPlayer.playFolderTrack(GENERAL_SOUND_FOLDER, HUM_SOUND);
    myDFPlayer.setRepeatPlayCurrentTrack(true);
  }

  void run() override {
    if (checkSwing()) {
      nextState = &swingState;
      isSoundDone = false;
    }
  }
} idleState;

void igniteSaber() {
  myDFPlayer.setVolume(0);  // DEBUG
  curVolume = 0;            // DEBUG
  goalVolume = 0;           // DEBUG
  myDFPlayer.setRepeatPlayCurrentTrack(false);
  myDFPlayer.playFolderTrack(IGNITE_SOUND_FOLDER, random(1, NUM_IGNITE_SOUND + 1));
}

class OffState : public State {
public:
  OffState()
    : State(OFF_STATE) {}

  void init() override {
    // TODO Create a power saving mode when "off"
    // TODO ignite only if button pressed
    Serial.println("State: OFF");
    igniteSaber();
  }

  void run() override {
    if (isSoundDone) {
      nextState = &idleState;
      isSoundDone = false;
    }
  }
} offState;

class StartState : public State {
public:
  StartState()
    : State(START_STATE) {}

  void init() override {
    Serial.println("START");
  }

  void run() override {
  }
} startState;


void updateVolume() {
  if (abs(curVolume - goalVolume) > 0.4) {
    if (curVolume < goalVolume) {  // Increasing volume
      curVolume = goalVolume * VOLUME_LERP + curVolume * (1 - VOLUME_LERP);
    } else {  // Decreasing volume
      curVolume = goalVolume * VOLUME_LERP * 2 + curVolume * (1 - VOLUME_LERP * 2);
    }
    Serial.println(curVolume);
    myDFPlayer.setVolume((int)curVolume);
  }
}

void loop() {
#ifdef LS_LOOPLENGHT
  Serial.println(millis() - loopcurrenttime);
  loopcurrenttime = millis();
#endif

  updateVolume();
  myDFPlayer.loop();

  if (resetSaber) {
    nextState = &idleState;
  }

  if (curState->getID() != nextState->getID()) {
    curState = nextState;
    curState->init();
  }

  curState->run();
}

void setupStates() {
  curState = &startState;
  nextState = &offState;
}

// TODO Implement
void setupLED() {
  return 0;

  // for (int i = NUM_PIXELS; i > -1; i--)
  // {
  //   pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  //   pixels.show();
  // }
}

void setupIMU() {
  mpu.initialize();
  Serial.println(
    mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize_light();

  // These values are hard coded for the IMU
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

void setupAudio() {
  myDFPlayer.begin();
  myDFPlayer.setVolume(VOLUME);
  curVolume = VOLUME;
  goalVolume = VOLUME;

  Serial.println(F("DFPlayer Mini online."));
}

void setupGeneral() {
  randomSeed(analogRead(0));
  Serial.begin(115200);
  while (!Serial) {
  }

  Serial.println(F("General setup done."));
}

void setup() {
  setupGeneral();
  setupAudio();
  setupIMU();
  setupLED();
  setupStates();
}