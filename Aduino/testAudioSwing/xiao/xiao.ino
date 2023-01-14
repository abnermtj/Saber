#include "sample.h"  //sample file
#define AUDIO_PIN D7
#define VOLUME 24 
float i;         //sample play progress
float audioSpeed = 1;  //sample frequency
bool startAudio, prevStartAudio, done_trig1;
int sound_out;       //sound out PWM rate
byte sample_no = 1;  //select sample number
uint8_t curVolume = VOLUME;
uint8_t curVolume = VOLUME;

//-------------------------timer interrupt for sound----------------------------------
hw_timer_t *audioSampleTimer = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t ledstat = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux0);  // I assume this means cannot be interrupted
  if (done_trig1 == 1) {               //when trigger in
    i = i + audioSpeed;
    if (i >= 28800) {  // When the whole sample is played,28800 = 48KHz sampling * 0.6sec
      i = 0;
      done_trig1 = 0;
    }
  }
  lswingSample = (((pgm_read_byte(&(lSwingSound[(int)i * 2]))) | (pgm_read_byte(&(lSwingSound[(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  hswingSample = (((pgm_read_byte(&(hSwingSound[(int)i * 2]))) | (pgm_read_byte(&(hSwingSound[(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  humSample = (((pgm_read_byte(&(hSwingSound[(int)i * 2]))) | (pgm_read_byte(&(hSwingSound[(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  sound_out*= 5;
  ledcWrite(1, sound_out + 511);                                                                                                    //PWM output first arg is the channel attached via ledcAttachPin()
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void playAdvertisement(){
 // TODO
}

void playHumSound(){
 // TODO
}

void playIgniteSound(){
   // TODO
}

void loopCurrentTrack(bool){
  // TODO
  
}

void setVolume(int volume){
  curVolume = contrain(volume,0, 30);
}

void setupAudio() {
  pinMode(AUDIO_PIN, OUTPUT);  //sound_out PWM
  setVolume(VOLUME);

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution
  ledcAttachPin(AUDIO_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply AUDIO_PIN output

  audioSampleTimer = timerBegin(0, 3628, true);            // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &onTimer, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(audioSampleTimer);                      // enable audioSampleTimer

  Serial.println(F("Audio setup complete"));
}

void setup() {
  Serial.begin(115200);

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution
  ledcAttachPin(AUDIO_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply AUDIO_PIN output

  audioSampleTimer = timerBegin(0, 3628, true);            // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &onTimer, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(audioSampleTimer);                      // enable audioSampleTimer

  Serial.println("DONE SETUP");
}

void playAudio() {
  done_trig1 = 1;
    i = 0;
}
void loop() {
  //-------------------------trigger audio play via serial ---------------------------
  String arrivingdatabyte = Serial.readString();
  if (arrivingdatabyte == "y") {
    playAudio();
  }

  //-------------------------pitch setting----------------------------------
  audioSpeed = 1;

  //-------------------------sample change----------------------------------
  sample_no = 0;
}