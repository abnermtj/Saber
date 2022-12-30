#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "MPU9250.h"

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

#define SWING_SOUND_FOLDER 5
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
#define VOLUME 12  // From 0 to 30

#define GYR_SWING_THRESHOLD 5000
#define SWING_TIMEOUT_MS 20
// ---------------------------- STATES -------------------------------
#define START_STATE 1
#define IDLE_STATE 2
#define SWING_STATE 3

int curState, nextState;
// -------------------------------------------------------------------

SoftwareSerial mySoftwareSerial(MP3_RX_PIN, MP3_TX_PIN);
DFRobotDFPlayerMini myDFPlayer;

void setupAudio() {
  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial, true, false)) {  // Third param "false" disables starting POP sounds
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(VOLUME);
}
void setupIMU() {
  Wire.begin();
  delay(2000);        // Wait for mpu to initialize
  mpu.verbose(true);  // For debug
  mpu.setup(0x68);    // Set to i2c address of mpu
}

void setup() {
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);
  setupAudio();
  setupIMU();
  curState = IDLE_STATE;
  nextState = START_STATE;
}

void igniteSaber() {
  myDFPlayer.playFolder(IGNITE_SOUND_FOLDER, random(1, NUM_IGNITE_SOUND));
  myDFPlayer.waitAvailable();
}

void loop() {
  getIMU();


  if (curState != nextState) {
    if (DEBUG) {
      Serial.print(F("Changing from state "));
      Serial.print(curState);
      Serial.print(F(" to state "));
      Serial.println(nextState);
    }

    curState = nextState;

    switch (nextState) {
      case START_STATE:
        break;
      case IDLE_STATE:
        myDFPlayer.enableLoop();
        myDFPlayer.playFolder(GENERAL_SOUND_FOLDER, HUM_SOUND);
        break;
    }
  }

  switch (curState) {
    case START_STATE:
      igniteSaber();

      nextState = IDLE_STATE;
      if (DEBUG && myDFPlayer.available()) {
        printDFPlayerDetail(myDFPlayer.readType(), myDFPlayer.read());
      }
      break;

    case IDLE_STATE:
      //myDFPlayer.waitAvailable();
      if (checkSwing()) {
       playSwing();
      }
      break;

    case SWING_STATE:
      myDFPlayer.playFolder(GENERAL_SOUND_FOLDER, HUM_SOUND);
      myDFPlayer.waitAvailable();

      
      break;

    default:
      curState = START_STATE;
  }
  
}

bool checkSwing(){
  long gyrMag = sq(gyrX) + sq(gyrY) + sq(gyrZ);
  Serial.println(gyrMag);
  if (gyrMag > GYR_SWING_THRESHOLD) {
    return true;
  } 
  
  return false;
}

void playSwing() {
  myDFPlayer.playFolder(SWING_SOUND_FOLDER, random(1,NUM_SWING_SOUND));
}
void getIMU()
{
  if (mpu.update()) {
    accX = mpu.getAccX();
    accY = mpu.getAccY();
    accZ = mpu.getAccZ();
    gyrX = mpu.getGyroX();
    gyrY = mpu.getGyroY();
    gyrZ = mpu.getGyroZ();
  }
}

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