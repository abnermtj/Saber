// ---------------------------- PINOUT -------------------------------
#define AUDIO_PIN D7
#define BUTTON_PIN A0
#define LED_PIN D8

// ---------------------------- Libraries -------------------------------
#include "Arduino.h"
// #include "SoftwareSerial.h"
#include "Led.h"
#include "sample.h"
#include "smooth_swing_v2.h"
#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
#include "NonBlockingDelay.h"
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



// ---------------------------- MP3 -------------------------------
// #define GENERAL_SOUND_FOLDER 1
// #define BOOT_SOUND 1
// #define CHANGE_COLOR_SOUND 2
// #define DRAG_SOUND 3  // When a sabers contacts any surface
// #define HUM_SOUND 4
// #define LOCKUP_SOUND 5  // When two sabers continiously in contact

// #define RETRACT_SOUND_FOLDER 2
// #define NUM_RETRACT_SOUND 2

// #define IGNITE_SOUND_FOLDER 3
// #define NUM_IGNITE_SOUND 5

// #define CLASH_SOUND_FOLDER 4
// #define NUM_CLASH_SOUND 16

// #define SWING_SOUND_FOLDER 5
// #define NUM_SWING_SOUND 16

// // class Mp3Notify;
// // SoftwareSerial softSerial(MP3_RX_PIN, MP3_TX_PIN);  // RXpin, TXpin
// // typedef DFMiniMp3<SoftwareSerial, Mp3Notify> DfMp3;
// // DfMp3 myDFPlayer(softSerial);

// float curVolume = 0;
// uint8_t prevVolume = 0;
// float goalVolume = 0;
// uint16_t prevTrack = -1;

long i, i2, i3;          //sample play progress
int audioSpeed = 1;      //sample frequency
volatile int sound_out;  //sound out PWM rate
float cur_lswingVolume = 0;
float cur_hswingVolume = 0;
float cur_humVolume = 1;

// timer interrupt for sound
hw_timer_t* audioSampleTimer = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------- IMU -------------------------------
MPU6050 mpu;

// Quaternion curRotation;  // estimated rotational forces
// Quaternion prevRotation;
// static Quaternion prevOrientation;  // estimate of the angle in space
// static Quaternion curOrientation;
// VectorInt16 curAccel;
// VectorInt16 prevAccel;
// VectorInt16 curDeltAccel;
// VectorInt16 prevDeltAccel;
// VectorInt16 curGyro;
// long curGyrMag, maxGyrMag, avgGyrMag = 0;
// long maxGyrTimeStamp = 0;

// long last_swing_time = 0;

// volatile bool mpuInterruptDetected = false;
// uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
// uint16_t DMPpacketSize;  // expected DMP packet size (default  42 bytes)
// uint8_t mpuFifoBuffer[64];
// uint16_t mpuFifoCount;  // count of all bytes currently in FIFO

// bool dmpReady = false;
// I2Cdev i2ccomm;

bool dmpReady = false;         // set true if DMP init was successful
uint8_t mpuIntStatus;          // holds actual interrupt status byte from MPU
uint8_t devStatus;             // return status after each device operation (0 = success, !0 = error)
uint16_t exepectedPacketSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];        // FIFO storage buffer
VectorInt16 accel, gyro;       // [x, y, z]            gyro sensor measurements
I2Cdev i2ccomm;

bool smoothSwingEnabled = false;

// ---------------------------- SETTINGS -------------------------------
// #define DEBUG 1
// #define VOLUME 25  // From 0 to 30
#define MASTER_VOLUME 3
#define VOLUME_LERP 0.043
#define MIN_SWING_VOLUME 18
#define MAX_SWING_VOLUME 29
#define NUM_PIXELS 144
#define ACC_SWING_THRESHOLD 4000       // 1000 for smooth swing sensitvit
#define GYR_SLOW_SWING_THRESHOLD 9000  // TODO update this
#define SWING_TIMEOUT_MS 500
#define RESWING_TIMEOUT_MS 250
// #define VOLUME_LERP 0.043
#define CLASH_THRESHOLD 15

// ---------------------------- FSM STATES -------------------------------
#define START_STATE 0
#define OFF_STATE 1
#define IGNITE_STATE 2
#define RETRACT_STATE 3
#define IDLE_STATE 4
#define SWING_STATE 5
#define CLASH_STATE 6

#define SWING_STATE_TIME_MS 800
#define CLASH_STATE_TIME_MS 900

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
volatile bool isSoundDone = false;


// ---------------------------- DEBUG -------------------------------
#ifdef DEBUG
unsigned long loopcurrenttime;
#endif
// ----------------------- MP3 Player Feedback --------------------------------

// Called every 22050Hz to play 10bit audio sample little endian
void IRAM_ATTR audioISR() {
  // Cannot be interupted
  portENTER_CRITICAL_ISR(&timerMux0);

  if (smoothSwingEnabled) {  // Start audio playback
    long lswingArrSize = sizeof(lowSwingSound) / sizeof(lowSwingSound[0]);
    long hswingArrSize = sizeof(highSwingSound) / sizeof(highSwingSound[0]);
    long humArrSize = sizeof(humSound) / sizeof(humSound[0]);

    i = (i + audioSpeed) % (lswingArrSize / 2 - 1);
    i2 = (i2 + audioSpeed) % (hswingArrSize / 2 - 1);
    i3 = (i3 + audioSpeed) % (humArrSize / 2 - 1);

    int16_t lswingSample = ((int16_t)((pgm_read_byte(&(lowSwingSound[i * 2]))) | (pgm_read_byte(&(lowSwingSound[i * 2 + 1]))) << 8) >> 6);
    int16_t hswingSample = ((int16_t)((pgm_read_byte(&(highSwingSound[i2 * 2]))) | (pgm_read_byte(&(highSwingSound[i2 * 2 + 1]))) << 8) >> 6);
    int16_t humSample = ((int16_t)((pgm_read_byte(&(humSound[i3 * 2]))) | (pgm_read_byte(&(humSound[i3 * 2 + 1]))) << 8) >> 6);

    sound_out = (lswingSample * cur_lswingVolume + hswingSample * cur_hswingVolume + humSample * cur_humVolume) / (MaxSwingVolume + 1);
    sound_out = constrain(sound_out * MASTER_VOLUME, -511, 512);
    ledcWrite(1, sound_out + 511);  //Sets voltage level (0-1023) for PWM on chanel 1
  }

  // Allow be interrupts
  portEXIT_CRITICAL_ISR(&timerMux0);
}

// class Mp3Notify {
// public:
//   static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action) {
//     if (source & DfMp3_PlaySources_Sd) {
//       Serial.print("SD Card, ");
//     }
//     if (source & DfMp3_PlaySources_Usb) {
//       Serial.print("USB Disk, ");
//     }
//     if (source & DfMp3_PlaySources_Flash) {
//       Serial.print("Flash, ");
//     }
//     Serial.println(action);
//   }
//   static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode) {
//     // see DfMp3_Error for code meaning
//     Serial.println();
//     Serial.print("Com Error ");
//     Serial.println(errorCode);
//   }

//   static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track) {
//     Serial.print("Play finished for #");
//     Serial.println(track);

//     // Ignore duplicate audio done messages
//     if (track != prevTrack) {
//       isSoundDone = true;
//     }

//     prevTrack = track;
//   }
//   static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
//     PrintlnSourceAction(source, "online");
//   }
//   static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
//     PrintlnSourceAction(source, "inserted");
//   }
//   static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source) {
//     PrintlnSourceAction(source, "removed");
//   }
// };

// inline void dmpDataReady() {
//   mpuInterruptDetected = true;
// }

long clashStartTime;
class ClashState : public State {
public:
  ClashState()
    : State(CLASH_STATE) {}
  void init() override {
    // myDFPlayer.playAdvertisement(1510);
    Serial.println("State: Clash");
    // myDFPlayer.setRepeatPlayCurrentTrack(false);
    // myDFPlayer.playAdvertisement(1510);
    // myDFPlayer.playFolderTrack(CLASH_SOUND_FOLDER,random(1, 1 + NUM_CLASH_SOUND));
    isSoundDone = false;
    clashStartTime = millis();
  }

  void run() override {
    if (isSoundDone || (millis() - clashStartTime) > CLASH_STATE_TIME_MS) {
      Serial.println("exit: Clash");
      isSoundDone = false;
      resetSaber = true;
    }
    // TODO
  }

} clashState;



void clash() {
  Serial.println("CLASH detected");
  nextState = &clashState;
}

template<typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

class SwingState : public State {
public:
  long swingStartTime = 0;
  bool hasSwingingStopped = false;
  int8_t initialSwingDirX, initialSwingDirY;

  SwingState()
    : State(SWING_STATE) {}

  // Checks if the direction of the saber has changed by comparing the signs of the gyro readings.
  // bool checkSwingDirChange() {
  //   int8_t diff = 0;

  //   diff += (sign(curGyro.x) != initialSwingDirX) ? 1 : 0;
  //   diff += (sign(curGyro.z) != initialSwingDirY) ? 1 : 0;
  //   // Serial.print("Number of axis reversed:");
  //   // Serial.println(diff);

  //   if (diff >= 1) {
  //     return true;
  //   }
  //   return false;
  // }

  void init() override {
    Serial.println("State: Swing");
    // printQuaternion(curRotation);
    // if (maxGyrMag < GYR_SLOW_SWING_THRESHOLD) {  // TODO update this condition
    //   Serial.println("SLOW SWING");
    //   // myDFPlayer.playAdvertisement(random(1, NUM_SWING_SOUND + 1));
    // } else {
    //   Serial.println("FAST SWING");
    //   // myDFPlayer.playAdvertisement(random(1000, 1002));
    // }
    // initialSwingDirX = sign(curGyro.x);
    // initialSwingDirY = sign(curGyro.z);
    // swingStartTime = millis();
    // hasSwingingStopped = false;
  }
  void run() override {
    // Check is reswing in different direction
    // if (checkSwing(true) && checkSwingDirChange()) {
    //   Serial.println("RESWING");
    //   curVolume = curVolume + 4;
    //   goalVolume = curVolume;
    //   init();
    // }

    // // Adjust volume to saber swing speed
    // uint8_t adjusted_volume = (curGyrMag / 200) + MIN_SWING_VOLUME;
    // adjusted_volume = constrain(adjusted_volume, MIN_SWING_VOLUME, MAX_SWING_VOLUME);

    // if (!hasSwingingStopped) {
    //   hasSwingingStopped = curRotation.w * 1000 < 999;
    // }

    // if (((millis() - swingStartTime) > SWING_STATE_TIME_MS)) {
    //   Serial.println("EXITING SWING STATE");
    //   resetSaber = true;
    //   isSoundDone = false;
    //   // goalVolume = VOLUME;
    // } else if (hasSwingingStopped) {
    //   goalVolume = MIN_SWING_VOLUME;
    // } else {
    //   goalVolume = adjusted_volume;
    // }
  }

} swingState;

// inline void printQuaternion(Quaternion quaternion) {
//   Serial.print(F("\t\tQ\t\tw="));
//   Serial.print(quaternion.w * 1000);
//   Serial.print(F("\t\tx="));
//   Serial.print(quaternion.x);
//   Serial.print(F("\t\ty="));
//   Serial.print(quaternion.y);
//   Serial.print(F("\t\tz="));
//   Serial.println(quaternion.z);
// }

// inline void motionEngine() {
//   if (!dmpReady)
//     return;

//   mpuInterruptDetected = false;
//   mpuIntStatus = mpu.getIntStatus();  // INT_STATUS byte
//   mpuFifoCount = mpu.getFIFOCount();

//   // IF FIFO OVERFLOW
//   if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
//     // reset so we can continue cleanly
//     mpu.resetFIFO();
//     curDeltAccel.x = 0;
//     curDeltAccel.y = 0;
//     curDeltAccel.z = 0;
//   } else if (mpuIntStatus & 0x02) {
//     // wait for correct available data length, should be a VERY short wait
//     while (mpuFifoCount < DMPpacketSize) {
//       mpuFifoCount = mpu.getFIFOCount();
//     }

//     // read a packet from FIFO
//     mpu.getFIFOBytes(mpuFifoBuffer, DMPpacketSize);

//     // track FIFO count here in case there is > 1 packet available
//     // (this lets us immediately read more without waiting for an interrupt)
//     // mpuFifoCount -= DMPpacketSize;

//     // ORIENTATION
//     prevOrientation = curOrientation.getConjugate();
//     mpu.dmpGetQuaternion(&curOrientation, mpuFifoBuffer);

//     // ROTATION
//     prevRotation = curRotation;
//     curRotation = prevOrientation.getProduct(
//       curOrientation.getNormalized());

//     // ACCELERATION
//     prevAccel = curAccel;
//     mpu.dmpGetAccel(&curAccel, mpuFifoBuffer);
//     curDeltAccel.x = prevAccel.x - curAccel.x;
//     curDeltAccel.y = prevAccel.y - curAccel.y;
//     curDeltAccel.z = prevAccel.z - curAccel.z;

//     // GYRO
//     mpu.dmpGetGyro(&curGyro, mpuFifoBuffer);
//     curGyrMag = sq(curGyro.x) + sq(curGyro.z);
//     avgGyrMag = curGyrMag * 0.7 + avgGyrMag * (1 - 0.7);

//     if (millis() - maxGyrTimeStamp > 30) {
//       maxGyrMag = 0;
//     }
//     if (curGyrMag > maxGyrMag) {
//       maxGyrMag = curGyrMag;
//       maxGyrTimeStamp = millis();
//     }
//     // printQuaternion(curRotation);
//   }
// }

// bool checkSwing(bool isReswing) {
//   motionEngine();

//   if (!isReswing && (millis() - last_swing_time) < SWING_TIMEOUT_MS) {
//     // Serial.println("NO SWINING YET");
//     return false;
//   } else if (isReswing && (millis() - last_swing_time) < RESWING_TIMEOUT_MS) {
//     // Serial.println("NO RESWINING YET");
//     return false;
//   }

//   if (abs(curRotation.w * 1000) < 999) {  // some rotation movement have been initiated
//     if (
//       // General Swing
//       (abs(curDeltAccel.x) > ACC_SWING_THRESHOLD
//        or abs(curDeltAccel.z) > ACC_SWING_THRESHOLD
//        or abs(curDeltAccel.y) > ACC_SWING_THRESHOLD * 10)
//       // Reswing Horizontal
//       or (abs(curDeltAccel.x) > abs(curDeltAccel.z)
//           and (abs(prevDeltAccel.x) > ACC_SWING_THRESHOLD)
//           and ((prevDeltAccel.x > 0
//                   and curDeltAccel.x < -ACC_SWING_THRESHOLD
//                 or (prevDeltAccel.x < 0
//                     and curDeltAccel.x > ACC_SWING_THRESHOLD))))
//       // Reswing Vertical
//       or (abs(curDeltAccel.z) > abs(curDeltAccel.x)
//           and (abs(prevDeltAccel.z) > ACC_SWING_THRESHOLD)
//           and ((prevDeltAccel.z > 0
//                 and curDeltAccel.z < -ACC_SWING_THRESHOLD)
//                or (prevDeltAccel.z < 0
//                    and curDeltAccel.z > ACC_SWING_THRESHOLD)))
//            // must not be triggered by pure blade rotation (wrist rotation)
//            and not(
//              (abs(prevRotation.y * 1000 - curRotation.y * 1000) > abs(prevRotation.x * 1000 - curRotation.x * 1000))
//              and (abs(prevRotation.y * 1000 - curRotation.y * 1000) > abs(prevRotation.z * 1000 - curRotation.z * 1000)))) {

//       // Serial.print(F("Acceleration\tx="));
//       // Serial.print(curDeltAccel.x);
//       // Serial.print(F("\ty="));
//       // Serial.print(curDeltAccel.y);
//       // Serial.print(F("\tz="));
//       // Serial.print(curDeltAccel.z);
//       // Serial.print(F("\tcurRotation\tw="));
//       // Serial.print(curRotation.w * 1000);
//       // Serial.print(F("\t\tx="));
//       // Serial.print(curRotation.x);
//       // Serial.print(F("\t\ty="));
//       // Serial.print(curRotation.y);
//       // Serial.print(F("\t\tz="));
//       // Serial.println(curRotation.z);

//       last_swing_time = millis();
//       return true;
//     }
//   }

//   return false;
// }

class IdleState : public State {
public:
  IdleState()
    : State(IDLE_STATE) {}
  void init() override {
    Serial.println("State: IDLE");

    resetSaber = false;
    smoothSwingEnabled = true;
    // goalVolume = VOLUME;

    // myDFPlayer.playFolderTrack(GENERAL_SOUND_FOLDER, HUM_SOUND);
  }

  void run() override {
    if (isButtonPressed()) {
      smoothSwingEnabled = false;
      retractSaber();
    } else {
      isSoundDone = false;

      if (!dmpReady) return;
      // read a packet from MPU FIFO
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
        mpu.dmpGetGyro(&gyro, fifoBuffer);
        Vec3 gyroVec = Vec3(gyro.x, gyro.y, gyro.z);
        SB_Motion(gyroVec, false);
      }

      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      long gyrMag = (long)gx * gx + (long)gy * gy + (long)gz * gz;
      long accMag = (long)ax * ax + (long)ay * ay + (long)az * az;
      if (gyrMag > 45000000 && accMag > 1900000000) {
        // Serial.print("gyr: ");
        // Serial.print(gyrMag);
        // Serial.print(" ,accel: ");
        // Serial.println(accMag);
        clash();
      }
      UpdateVolume();
    }
  }
} idleState;

void igniteSaber() {
  // myDFPlayer.setVolume(0);  // DEBUG
  // curVolume = 0;   // DEBUG
  // goalVolume = 0;  // DEBUG
  // myDFPlayer.setRepeatPlayCurrentTrack(false);
  // myDFPlayer.playFolderTrack(IGNITE_SOUND_FOLDER, random(1, NUM_IGNITE_SOUND + 1));
  IgniteLED(strip.Color(50, 0, 0), 0.000002, strip.Color(255, 255, 255), 0, 50);
}


bool isButtonPressed() {
  int buttonVal = analogRead(BUTTON_PIN);
  // Serial.println(buttonVal);
  if (buttonVal == 255) {
    return true;
  }

  return false;
}


class IgniteState : public State {
public:
  IgniteState()
    : State(IGNITE_STATE) {}

  void init() override {
    Serial.println("State: Ignite");
  }

  void run() override {
    // if (isSoundDone) {
    delay(1000);  //debug

    // myDFPlayer.setRepeatPlayCurrentTrack(true);
    nextState = &idleState;
    isSoundDone = false;
    // breatheLED(80,60,3,1);
    // }
  }
} igniteState;

class OffState : public State {
public:
  OffState()
    : State(OFF_STATE) {}

  void init() override {
    // TODO Create a power saving mode when "off"
    // TODO ignite only if button pressed
    Serial.println("State: OFF");
  }

  void run() override {
    if (isButtonPressed()) {
      igniteSaber();
      nextState = &igniteState;
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


class RetractState : public State {
public:
  RetractState()
    : State(IGNITE_STATE) {}

  void init() override {
    Serial.println("State: Retract");
  }

  void run() override {
    // if (isSoundDone) {
    delay(1000);  //debug

    // myDFPlayer.setRepeatPlayCurrentTrack(true);
    nextState = &offState;
    isSoundDone = false;
    // }
  }
} retractState;

void retractSaber() {
  RetractLED(0.00001);
  nextState = &retractState;
}

// void updateVolume() {
//   if (curVolume < goalVolume) {  // Increasing volume
//     curVolume = goalVolume * VOLUME_LERP + curVolume * (1 - VOLUME_LERP);
//   } else {                                                                         // Decreasing volume
//     curVolume = goalVolume * VOLUME_LERP * 2 + curVolume * (1 - VOLUME_LERP * 2);  // Fast to go silent to taper swings
//   }

//   if (int(curVolume) != prevVolume) {
//     // Serial.println(int(curVolume));
//     // myDFPlayer.setVolume((int)curVolume);
//     prevVolume = curVolume;
//   }
// }
void UpdateVolume() {
  if (cur_lswingVolume < lswingVolume) {  // Increasing volume is faster than decreasing volume
    cur_lswingVolume = lswingVolume * VOLUME_LERP * 2 + cur_lswingVolume * (1 - VOLUME_LERP * 2);
  } else {                                                                                 // Decreasing volume
    cur_lswingVolume = lswingVolume * VOLUME_LERP + cur_lswingVolume * (1 - VOLUME_LERP);  // Fade Out
  }

  if (cur_hswingVolume < hswingVolume) {  // Increasing volume is faster than decreasing volume
    cur_hswingVolume = hswingVolume * VOLUME_LERP * 2 + cur_hswingVolume * (1 - VOLUME_LERP * 2);
  } else {                                                                                 // Decreasing volume
    cur_hswingVolume = hswingVolume * VOLUME_LERP + cur_hswingVolume * (1 - VOLUME_LERP);  // Fade Out
  }

  if (cur_humVolume < humVolume) {  // Increasing volume is faster than decreasing volume
    cur_humVolume = humVolume * VOLUME_LERP * 2 + cur_humVolume * (1 - VOLUME_LERP * 2);
  } else {                                                                        // Decreasing volume
    cur_humVolume = humVolume * VOLUME_LERP + cur_humVolume * (1 - VOLUME_LERP);  // Fade Out
  }
}

void loop() {
#ifdef DEBUG
  // Serial.println(millis() - loopcurrenttime);
  loopcurrenttime = millis();
#endif


  // myDFPlayer.loop();

  if (resetSaber) {
    nextState = &idleState;
  }

  if (curState->getID() != nextState->getID()) {
    curState = nextState;
    curState->init();
  }

  curState->run();

  // Serial.println(sound_out);
}

void setupStates() {
  curState = &startState;
  nextState = &offState;
}

void initIMUOffsets() {
  mpu.setXAccelOffset(-3289);
  mpu.setYAccelOffset(-1392);
  mpu.setZAccelOffset(516);
  mpu.setXGyroOffset(240);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(27);
}

void setupIMU() {
  // mpu.initialize();
  // Serial.println(
  //   mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  // Serial.println(F("Initializing DMP..."));
  // uint8_t devStatus = mpu.dmpInitialize_light();

  // // These values are hard coded for the IMU
  // mpu.setXAccelOffset(-3289);
  // mpu.setYAccelOffset(-1392);
  // mpu.setZAccelOffset(516);
  // mpu.setXGyroOffset(240);
  // mpu.setYGyroOffset(9);
  // mpu.setZGyroOffset(27);

  // if (devStatus == 0) {
  //   mpu.setDMPEnabled(true);

  //   attachInterrupt(0, dmpDataReady, RISING);
  //   mpuIntStatus = mpu.getIntStatus();

  //   dmpReady = true;
  //   DMPpacketSize = mpu.dmpGetFIFOPacketSize();
  //   Serial.println(F("DMP READY"));
  // }

  // // configure the motion interrupt for clash recognition
  // // INT_PIN_CFG register
  // mpu.setDLPFMode(3);  // Digital Low-Pass Frequenct
  // mpu.setDHPFMode(0);  // Digital High-Pass Frequency
  // //mpu.setFullScaleAccelRange(3);
  // mpu.setIntMotionEnabled(true);
  // mpu.setIntZeroMotionEnabled(false);
  // mpu.setIntFIFOBufferOverflowEnabled(false);
  // mpu.setIntI2CMasterEnabled(false);
  // mpu.setIntDataReadyEnabled(false);
  // mpu.setMotionDetectionThreshold(CLASH_THRESHOLD);  // 1mg/LSB
  // mpu.setMotionDetectionDuration(2);                 // number of consecutive samples above threshold to trigger int


  // // configure Interrupt with:
  // // int level active low
  // // int driver open drain
  // // interrupt latched until read out (not 50us pulse)
  // i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x37, 0xF0);
  // // enable only Motion Interrupt
  // i2ccomm.writeByte(MPU6050_DEFAULT_ADDRESS, 0x38, 0x40);
  // mpuIntStatus = mpu.getIntStatus();

  // pinMode(MPU_INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(0, ISR_MPUInterrupt, FALLING);  // ISR
  // Serial.println(F("IMU intterupts READY"));
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // supply your own gyro offsets here, scaled for min sensitivity
  Serial.println(F("Initializing IMU offsets"));
  initIMUOffsets();

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  if (devStatus == 0) {
    Serial.println(F("DMP initialization success, Enabling DMP..."));
    mpu.setDMPEnabled(true);

    dmpReady = true;
    exepectedPacketSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed (usualy case)
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setupAudio() {
  // myDFPlayer.begin();
  // myDFPlayer.setVolume(VOLUME);
  // curVolume = VOLUME;
  // goalVolume = VOLUME;

  pinMode(AUDIO_PIN, OUTPUT);

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution in bits
  ledcAttachPin(AUDIO_PIN, 1);  // Set Audio pin to timer ch1

  audioSampleTimer = timerBegin(0, 3628, true);             // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &audioISR, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);               // Counts up to 1 then triggers the interrupt, auto-reload
  timerAlarmEnable(audioSampleTimer);                       // enable audioSampleTimer

  Serial.println(F("Audio Setup done."));
}

void setupGeneral() {
  pinMode(BUTTON_PIN, INPUT);
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
  SmoothSwingInit();
}