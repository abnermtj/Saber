#include "sample.h"  //sample file
#define SOUND_OUT_PIN D7

float i;         //sample play progress
float audioSpeed = 1;  //sample frequency
bool startAudio, prevStartAudio, done_trig1;
int sound_out;       //sound out PWM rate
byte sample_no = 1;  //select sample number

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
  sound_out = (((pgm_read_byte(&(smpl[(int)i * 2]))) | (pgm_read_byte(&(smpl[(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  sound_out*= 5;
  ledcWrite(1, sound_out + 511);                                                                                                    //PWM output first arg is the channel attached via ledcAttachPin()
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  Serial.begin(115200);
  sample_no++;               //countermeasure rotary encoder error
  if (sample_no >= 48) {     //countermeasure rotary encoder error
    sample_no = 0;
  }

  pinMode(SOUND_OUT_PIN, OUTPUT);  //sound_out PWM

  ledcSetup(1, 39000, 10);          //ledchannel, PWM frequency, resolution
  ledcAttachPin(SOUND_OUT_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply SOUND_OUT_PIN output

  //uint8_t timer_num, uint16_t divider, bool countUp
  // Assume 80Mhz
  audioSampleTimer = timerBegin(0, 1666, true);            // 12.5ns*1666 = 20.83usec(48kHz), count-up
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