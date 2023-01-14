#include "sample.h"
#include "smooth_swing_v2.h"

#define AUDIO_PIN D7
#define VOLUME 7 
int i, i2, i3;       //sample play progress
int audioSpeed = 1;  //sample frequency
bool startAudio, prevStartAudio, done_trig1;
int sound_out;  //sound out PWM rate


//-------------------------timer interrupt for sound----------------------------------
hw_timer_t *audioSampleTimer = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t ledstat = 0;

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
VectorInt16 raw_gyro;         // [x, y, z]            gyro sensor measurements
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



void IRAM_ATTR onTimer() {
  // Cannot be interupted
  portENTER_CRITICAL_ISR(&timerMux0);

  if (done_trig1 == 1) {  // Start audio playback



    int lswingArrSize = sizeof(lowSwingSound) / sizeof(lowSwingSound[0]);
    int hswingArrSize = sizeof(highSwingSound) / sizeof(highSwingSound[0]);
    int humArrSize = sizeof(humSound) / sizeof(humSound[0]);

    i = (i + audioSpeed) % lswingArrSize;
    i2 = (i2 + audioSpeed) % hswingArrSize;
    i3 = (i3 + audioSpeed) % humArrSize;

    int lswingSample = (((pgm_read_byte(&(lowSwingSound[(int)i * 2]))) | (pgm_read_byte(&(lowSwingSound[(int)i * 2 + 1]))) << 8) >> 6);      //16bit to 10bit
    int hswingSample = (((pgm_read_byte(&(highSwingSound[(int)i2 * 2]))) | (pgm_read_byte(&(highSwingSound[(int)i2 * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
    int humSample = (((pgm_read_byte(&(humSound[(int)i3 * 2]))) | (pgm_read_byte(&(humSound[(int)i3 * 2 + 1]))) << 8) >> 6);                 //16bit to 10bit

    sound_out = (lswingSample * lswingVolume + hswingSample * hswingVolume + humSample * humVolume )* VOLUME;
    ledcWrite(1, sound_out + 511);  //PWM output first arg is the channel attached via ledcAttachPin()
  }

  // Allow be interrupts
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void playAdvertisement() {
  // TODO
}

void playHumSound() {
  // TODO
}

void playIgniteSound() {
  // TODO
}

void loopCurrentTrack(bool) {
  // TODO
}

void setupAudio() {
  pinMode(AUDIO_PIN, OUTPUT);  //sound_out PWM

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution
  ledcAttachPin(AUDIO_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply AUDIO_PIN output

  audioSampleTimer = timerBegin(0, 3628, true);            // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &onTimer, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(audioSampleTimer);                      // enable audioSampleTimer

  Serial.println(F("Audio setup complete"));
}

void setupIMU() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
void setup() {
  Serial.begin(115200);

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution
  ledcAttachPin(AUDIO_PIN, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply AUDIO_PIN output

  audioSampleTimer = timerBegin(0, 3628, true);            // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &onTimer, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(audioSampleTimer);                      // enable audioSampleTimer

  setupIMU();
  SB_Init();
  done_trig1 = 1;
  Serial.println("DONE SETUP");
}


void loop() {
  //-------------------------pitch setting----------------------------------
  audioSpeed = 1;


   // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
   mpu.dmpGetGyro(&raw_gyro, fifoBuffer);
  }
  Vec3 raw_gyro_converted =  Vec3(raw_gyro.x, raw_gyro.y, raw_gyro.z);
  SB_Motion(raw_gyro_converted, false);
}