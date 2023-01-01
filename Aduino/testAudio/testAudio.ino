#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFMiniMp3.h"

// ---------------------------- PINOUT -------------------------------
#define MP3_RX_PIN 10
#define MP3_TX_PIN 11

class Mp3Notify;
SoftwareSerial softSerial(MP3_RX_PIN, MP3_TX_PIN);  // RXpin, TXpin
typedef DFMiniMp3<SoftwareSerial, Mp3Notify> DfMp3;
DfMp3 myDFPlayer(softSerial);

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

void setup() {
  softSerial.begin(9600);
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  myDFPlayer.begin();
  
 
  
  myDFPlayer.setRepeatPlayCurrentTrack(false);
  myDFPlayer.setVolume(25);  //Set volume value. From 0 to 30
  myDFPlayer.playFolderTrack(1, 4);
Serial.println(F("DFPlayer Mini online."));
}

int dir = -1;
int volume = 15;
void loop() {
  myDFPlayer.loop();
  static unsigned long timer = millis();
  // Serial.println(volume);
  // if (millis() - timer > 100) {
  //   timer = millis();
  //   volume += dir;

  //   if (volume > 30) {
  //     volume = 30;
  //     dir = -1;
  //   }
  //   if (volume < 15) {
  //     volume = 15;
  //     dir = 1;
  //   }
  //   myDFPlayer.setVolume(volume);
  // }

}