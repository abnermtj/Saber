#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFMiniMp3.h"
#include "MPU9250.h"
#include "NonBlockingDelay.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// ---------------------------- DEBUG -------------------------------
#define LS_LOOPLENGHT
#ifdef LS_LOOPLENGHT
unsigned long loopcurrenttime;
#endif


// ---------------------------- PINOUT -------------------------------
#define MP3_RX_PIN 10
#define MP3_TX_PIN 11

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
MPU9250 mpu;

float accX, accY, accZ;
float gyrX, gyrY, gyrZ;

unsigned long swing_timer;
long last_swing_time = 0;


// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1
#define VOLUME 24  // From 0 to 30
#define MIN_SWING_VOLUME 5
#define MAX_SWING_VOLUME 30
#define NUM_PIXELS 144
#define GYR_SWING_THRESHOLD 4000
#define GYR_RESWING_THRESHOLD 10000
#define GYR_SLOW_SWING_THRESHOLD 9000
#define SWING_TIMEOUT_MS 500
#define RESWING_TIMEOUT_MS 400
#define VOLUME_LERP 0.04
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

long curGyrMag, maxGyrMag, avgGyrMag = 0;
long maxGyrTimeStamp = 0;
void getIMU() {
  if (mpu.update()) {
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
    gyrX = mpu.getGyroX();
    gyrY = mpu.getGyroY();
    // gyrZ = mpu.getGyroZ();
    gyrZ = 0;

    curGyrMag = sq(gyrX) + sq(gyrY) + sq(gyrZ);
    avgGyrMag = curGyrMag * 0.7 + avgGyrMag * (1 - 0.7);

    if (millis() - maxGyrTimeStamp > 30) {
      maxGyrMag = 0;
    }

    if (curGyrMag > maxGyrMag) {
      maxGyrMag = curGyrMag;
      maxGyrTimeStamp = millis();
    }
  }
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


// TODO https://therebelarmory.com/thread/9138/smoothswing-v2-algorithm-description
class SwingState : public State {
public:
  int swingCountDown = 8;
  long swingStateTime = 0;
  int initialSwingDirX, initialSwingDirY, initialSwingDirZ;

  SwingState()
    : State(SWING_STATE) {}

  //TODO
  bool checkDir() {
    int diff = 0;
    if (sign(gyrX) != initialSwingDirX) {
      diff++;
    }
    if (sign(gyrY) != initialSwingDirY) {
      diff++;
    }
    // if (sign(gyrZ) != initialSwingDirZ) {
    //   diff++;
    // }
    // Serial.print("Number of axis reversed:");
    // Serial.println(diff);

    if (diff >= 1) {
      return true;
    }

    return false;
  }

  bool checkReSwing() {
    if ((millis() - last_swing_time) < RESWING_TIMEOUT_MS) {
      return false;
    }

    getIMU();

    if (curGyrMag > GYR_RESWING_THRESHOLD) {
      last_swing_time = millis();
      return true;
    }

    return false;
  }

  void init() override {
    Serial.println("SWING");
    // myDFPlayer.setRepeatPlayCurrentTrack(false);
    for (int i = 0; i < swingCountDown; i++) {
      getIMU();  // 3 ms each loop!
    }
    Serial.println(maxGyrMag);
    if (maxGyrMag < GYR_SLOW_SWING_THRESHOLD) {
      // Serial.println("SLOW SWING");
      myDFPlayer.playAdvertisement(random(1, NUM_SWING_SOUND + 1));
    } else {
      // Serial.println("FAST SWING");
      myDFPlayer.playAdvertisement(random(1000, 1002));
    }

    initialSwingDirX = sign(gyrX);
    initialSwingDirY = sign(gyrY);
    initialSwingDirZ = sign(gyrZ);
    swingStateTime = millis();
  }
  void run() override {

    // Check is reswing in different direction
    if (checkReSwing() && checkDir()) {
      Serial.println("RESWING");
      init();
    }
    // Adjust volume to saber swing speed
    int adjusted_volume = (avgGyrMag / 200) + MIN_SWING_VOLUME;
    adjusted_volume = constrain(adjusted_volume, MIN_SWING_VOLUME, MAX_SWING_VOLUME);

    // Serial.print("avgGyrMag: ");
    // Serial.print(avgGyrMag);
    // Serial.print(" adjusted_volume:");
    // Serial.println(adjusted_volume);
    if (((millis() - swingStateTime) > SWING_STATE_TIME_MS)
        || (adjusted_volume == MIN_SWING_VOLUME)) {
      Serial.println("RESET");
      resetSaber = true;
      isSoundDone = false;
      goalVolume = VOLUME;
      return;
    } else {
      goalVolume = adjusted_volume;
    }
  }

} swingState;

bool checkSwing() {
  if ((millis() - last_swing_time) < SWING_TIMEOUT_MS) {
    return false;
  }

  getIMU();

  if (curGyrMag > GYR_SWING_THRESHOLD) {
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

    Serial.println("IDLE");
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
    Serial.println("OFF");
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
    if (curVolume < goalVolume) { // Increasing volume
      curVolume = goalVolume * VOLUME_LERP + curVolume * (1 - VOLUME_LERP);
    } else { // Decreasing volume
      curVolume = goalVolume * VOLUME_LERP * 2 + curVolume * (1 - VOLUME_LERP * 2);
    }
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
  Wire.begin();
  delay(1500);        // Wait for mpu to initialize
  mpu.verbose(true);  // Debug
  mpu.setup(0x68);    // Set to i2c address of mpu

  Serial.println(F("IMU online."));
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

  Serial.println(F("General setup."));
}

void setup() {
  setupGeneral();
  setupAudio();
  setupIMU();
  setupLED();
  setupStates();
}