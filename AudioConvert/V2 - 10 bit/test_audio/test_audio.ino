//#define  ENCODER_OPTIMIZE_INTERRUPTS //rotary encoder
#include "sample.h"  //sample file
#include <EEPROM.h>
#define BUTTON_PIN D7
#define SOUND_OUT_PIN D5

float oldPosition = -999;  //rotary encoder
float newPosition = -999;  //rotary encoder

float i;         //sample play progress
float freq = 1;  //sample frequency
bool trig1, old_trig1, done_trig1;
int sound_out;       //sound out PWM rate
byte sample_no = 1;  //select sample number

long timer = 0;         //timer count for eeprom write
bool eeprom_write = 0;  //0=no write,1=write

//-------------------------timer interrupt for sound----------------------------------
hw_timer_t *timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t ledstat = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux0);  // I assume this means cannot be interrupted
  if (done_trig1 == 1) {               //when trigger in
    i = i + freq;
    if (i >= 28800) {  // When the whole sample is played,28800 = 48KHz sampling * 0.6sec
      i = 0;
      done_trig1 = 0;
    }
  }
  sound_out = (((pgm_read_byte(&(smpl[sample_no][(int)i * 2]))) | (pgm_read_byte(&(smpl[sample_no][(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  sound_out*= 5;
  ledcWrite(1, sound_out + 511);                                                                                                    //PWM output first arg is the channel attached via ledcAttachPin()
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(1);           //1byte memory space
  EEPROM.get(0, sample_no);  // gets data from emprom address and puts into sample_no
  sample_no++;               //countermeasure rotary encoder error
  if (sample_no >= 48) {     //countermeasure rotary encoder error
    sample_no = 0;
  }

  pinMode(BUTTON_PIN, INPUT);      //trigger in
  pinMode(SOUND_OUT_PIN, OUTPUT);  //sound_out PWM
  timer = millis();                //for eeprom write
  analogReadResolution(10);

  ledcSetup(1, 39000, 10);          //ledchannel, PWM frequency, resolution
  ledcAttachPin(SOUND_OUT_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply SOUND_OUT_PIN output

  //uint8_t timer_num, uint16_t divider, bool countUp
  // Assume 80Mhz
  timer0 = timerBegin(0, 1666, true);            // timer0, 12.5ns*1666 = 20.83usec(48kHz), count-up
  timerAttachInterrupt(timer0, &onTimer, true);  // edge-triggered
  timerAlarmWrite(timer0, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(timer0);                      // enable timer0
  Serial.println("DONE SETUP");
}

void loop() {
  //-------------------------trigger button press ---------------------------
  old_trig1 = trig1;
  trig1 = digitalRead(BUTTON_PIN);
  String arrivingdatabyte = Serial.readString();
  if (arrivingdatabyte == "y") {
    Serial.println("HERE");
    trig1 = 1;
  } else {
    trig1 = 0;
  }
  if (trig1 == 1 && old_trig1 == 0) {  //detect trigger signal low to high , before sample play was done
    done_trig1 = 1;
    i = 0;
  }

  //-------------------------pitch setting----------------------------------
  // freq = analogRead(A3) * 0.002 + analogRead(A0) *  0.002;
  freq = 1;

  //-------------------------sample change----------------------------------
  newPosition = 5;
  sample_no = 0;
  // if ((newPosition - 3) / 4 > oldPosition / 4) {  // CLOCKWISE
  //   oldPosition = newPosition;
  //   sample_no = sample_no - 1;
  //   if (sample_no < 0 || sample_no > 200) {  //>200 is overflow countermeasure
  //     sample_no = 47;
  //   }
  //   done_trig1 = 1;  //1 shot play when sample changed
  //   i = 0;
  //   timer = millis();
  //   eeprom_write = 1;  //eeprom update flug on
  // }

  // else if ((newPosition + 3) / 4 < oldPosition / 4) {  // ANTI- CLOCKWISE
  //   oldPosition = newPosition;
  //   sample_no = sample_no + 1;
  //   if (sample_no >= 48) {
  //     sample_no = 0;
  //   }
  //   done_trig1 = 1;  //1 shot play when sample changed
  //   i = 0;
  //   timer = millis();
  //   eeprom_write = 1;  //eeprom update flug on
  // }

  //-------------------------save to eeprom----------------------------------
  if (timer + 5000 <= millis() && eeprom_write == 1) {  //Memorized 5 seconds after sample number change
    eeprom_write = 0;
    eeprom_update();
  }
}

void eeprom_update() {
  EEPROM.put(0, sample_no);
  EEPROM.commit();
}