#include "Arduino.h"
#include "SoftwareSerial.h"
#include <DFMiniMp3.h>
#include "MPU9250.h"
#include "NonBlockingDelay.h"

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


// ---------------------------- IMU -------------------------------
MPU9250 mpu;

float accX, accY, accZ;
float gyrX, gyrY, gyrZ;
unsigned long accMag, gyrMag;

unsigned long swing_timer;


// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1
#define VOLUME 30  // From 0 to 30

#define GYR_SWING_THRESHOLD 500
#define SWING_TIMEOUT_MS 70

// ---------------------------- STATES -------------------------------
#define OFF_STATE 0
#define IDLE_STATE 1
#define SWING_STATE 2
#define CLASH_STATE 3

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
bool isIgniteDone = false;
bool isSwingDone = false;
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

    // TODO replace with is audio done
    isSwingDone = true;
    switch (track) {
      case 6:
        isIgniteDone = true;
        break;
      default:
        break;
    }
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


void getIMU() {
  if (mpu.update()) {
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
    gyrX = mpu.getGyroX();
    gyrY = mpu.getGyroY();
    gyrZ = mpu.getGyroZ();
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

// https://therebelarmory.com/thread/9138/smoothswing-v2-algorithm-description
class SwingState : public State {
public:
  SwingState()
    : State(SWING_STATE) {}
  void init() override {
    Serial.println("HERE");
    myDFPlayer.setRepeatPlayCurrentTrack(false);
    myDFPlayer.playFolderTrack(SWING_SOUND_FOLDER, random(1, NUM_SWING_SOUND));

  }

  bool checkDir() {
    if (curDir != prevDir){
      return true;
    }

    angle = std::acos(dot(v1,v2)/(mag(v1)*mag(v2))
    return false;
  }

  void run() override {
    //  Serial.println("isSwingDone");
    //Serial.println(isSwingDone);
    //  Serial.println("heckSwing()");
   // Serial.println(checkSwing());
    
    if (checkSwing() && checkDir()) {
      init();
    }
    else if (isSwingDone) {
      resetSaber = true;
      isSwingDone = false;
    }
  }
} swingState;


long avgGyrMag = 0;
long last_swing_time = 0;
bool checkSwing() {
  if ((millis() - last_swing_time) < SWING_TIMEOUT_MS){
    return false;
  }

  getIMU();

  long gyrMag = sq(gyrX) + sq(gyrY) + sq(gyrZ);
  avgGyrMag = gyrMag * 0.7 + avgGyrMag * (1 - 0.7);
  //Serial.println(avgGyrMag);
  if (gyrMag > GYR_SWING_THRESHOLD) {
    last_swing_time = millis();
    return true;
  }

  return false;
}

void igniteSaber() {
  myDFPlayer.setVolume(0);
  myDFPlayer.playFolderTrack(IGNITE_SOUND_FOLDER, random(1, NUM_IGNITE_SOUND));
}

class IdleState : public State {
public:
  IdleState()
    : State(IDLE_STATE) {}
  void init() override {
      myDFPlayer.setVolume(VOLUME);
    myDFPlayer.playFolderTrack(GENERAL_SOUND_FOLDER, HUM_SOUND);
    myDFPlayer.setRepeatPlayCurrentTrack(true);

    resetSaber = false;
  }

  void run() override {
    
    if (checkSwing()) {
      myDFPlayer.setRepeatPlayCurrentTrack(false);
      nextState = &swingState;
      isSwingDone = false;
    }
  }
} idleState;

class OffState : public State {
public:
  OffState()
    : State(OFF_STATE) {}

  void init() override {
    // TODO Create a power saving mode when "off"
    igniteSaber();
  }

  void run() override {
    if (isIgniteDone) {
      nextState = &idleState;
    }
  }
} offState;

void loop() {
  if (resetSaber) {
    nextState = &idleState;
  }

  myDFPlayer.loop();

  if (curState->getID() != nextState->getID()) {
    Serial.print(F("Changing from state "));
    Serial.print((int)curState->getID());
    Serial.print(F(" to state "));
    Serial.println((int)nextState->getID());

    curState = nextState;
    curState->init();
  }

  curState->run();
}

void setupStates() {
  curState = &idleState;
  nextState = &offState;
}

void setupIMU() {
  Wire.begin();
  delay(2000);        // Wait for mpu to initialize
  mpu.verbose(true);  // For debug
  mpu.setup(0x68);    // Set to i2c address of mpu
}

void setupAudio() {
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  myDFPlayer.begin();

  myDFPlayer.setVolume(VOLUME);
  Serial.println(F("DFPlayer Mini online."));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }

  setupAudio();
  setupIMU();
  setupStates();
}