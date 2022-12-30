#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "MPU9250.h"
#include "NonBlockingDelay.h"

// ---------------------------- MP3 Directory -------------------------------
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

#define SWING_SOUND_FOLDER 4
#define NUM_SWING_SOUND 16

// ---------------------------- IMU -------------------------------
MPU9250 mpu;

float accX, accY, accZ;
float gyrX, gyrY, gyrZ;
unsigned long accMag, gyrMag;

unsigned long swing_timer;

// ---------------------------- PINOUT -------------------------------
#define MP3_RX_PIN 10
#define MP3_TX_PIN 11

// ---------------------------- SETTINGS -------------------------------
#define DEBUG 1
#define VOLUME 20  // From 0 to 30

#define GYR_SWING_THRESHOLD 500
#define SWING_TIMEOUT_MS 20
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

// -------------------------------------------------------------------

SoftwareSerial mySoftwareSerial(MP3_RX_PIN, MP3_TX_PIN);
DFRobotDFPlayerMini myDFPlayer;

//Print the detail message from DFPlayer to handle different errors and states.
void printDFPlayerDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

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

class SwingState : public State {
public:
  SwingState()
    : State(SWING_STATE) {}
  void init() override {
    myDFPlayer.disableLoop();
    myDFPlayer.playFolder(SWING_SOUND_FOLDER, random(1, NUM_SWING_SOUND));
    myDFPlayer.waitAvailable();
    resetSaber = true;
  }

  void run() override {
    if (checkSwing()) {
      init();
    }
  }
} swingState;


long avgGyrMag = 0;

bool checkSwing() {
  getIMU();
  
  long gyrMag = sq(gyrX) + sq(gyrY) + sq(gyrZ);
  avgGyrMag = gyrMag * 0.7 + avgGyrMag * (1 - 0.7); 
  Serial.println(avgGyrMag);
  if (gyrMag > GYR_SWING_THRESHOLD) {
    return true;
  }

  return false;
}

void igniteSaber() {
 // myDFPlayer.playFolder(IGNITE_SOUND_FOLDER, random(1, NUM_IGNITE_SOUND));
  myDFPlayer.waitAvailable();
}

class IdleState : public State {
public:
  IdleState()
    : State(IDLE_STATE) {}
  void init() override {
    myDFPlayer.enableLoop();
    myDFPlayer.playFolder(GENERAL_SOUND_FOLDER, HUM_SOUND);
  }

  void run() override {
    if (checkSwing()) {
      nextState = &swingState;
    }

    if (DEBUG && myDFPlayer.available()) {
      printDFPlayerDetail(myDFPlayer.readType(), myDFPlayer.read());
    }
  }
} idleState;

class OffState : public State {
public:
  OffState()
    : State(OFF_STATE) {}

  void init() override {
    // TODO Create a power saving mode when "off"
    // set_sleep_mode(SLEEP_MODE_IDLE);

    // power_adc_disable();
    // power_spi_disable();
    // power_timer0_disable();
    // power_timer1_disable();
    // power_timer2_disable();
    // power_twi_disable();

    // sleep_enable();
    // sleep_mode();
    myDFPlayer.disableLoop();
    igniteSaber();
    nextState = &idleState;
  }
} offState;

void loop() {
  if (resetSaber) {
    nextState = &idleState;
    resetSaber = false;
  }  
  
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

  if (!myDFPlayer.begin(mySoftwareSerial, true, false)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
      ;
  }

  myDFPlayer.volume(VOLUME);
  Serial.println(F("DFPlayer Mini online."));
}

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  setupAudio();
  setupIMU();
  setupStates();
}