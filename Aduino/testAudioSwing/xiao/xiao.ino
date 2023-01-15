#include "sample.h"
#include "smooth_swing_v2.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// ---------------------------- PINOUT -------------------------------
#define AUDIO_PIN D7

// ---------------------------- MP3 -------------------------------
long i, i2, i3;      //sample play progress
int audioSpeed = 1;  //sample frequency
int sound_out;       //sound out PWM rate
int idleState;
float cur_lswingVolume = 0;
float cur_hswingVolume = 0;
float cur_humVolume = 1;

// ---------------------------- SETTINGS -------------------------------
#define MASTER_VOLUME 3
#define VOLUME_LERP 0.043

//-------------------------timer interrupt for sound----------------------------------
hw_timer_t *audioSampleTimer = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;


// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;         // set true if DMP init was successful
uint8_t mpuIntStatus;          // holds actual interrupt status byte from MPU
uint8_t devStatus;             // return status after each device operation (0 = success, !0 = error)
uint16_t exepectedPacketSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];        // FIFO storage buffer
VectorInt16 accel, gyro;       // [x, y, z]            gyro sensor measurements
I2Cdev i2ccomm;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// Called every 22050Hz to play 10bit audio sample little endian
void IRAM_ATTR audioISR() {
  // Cannot be interupted
  portENTER_CRITICAL_ISR(&timerMux0);

  if (idleState == 1) {  // Start audio playback
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

void clash() {
  Serial.println("CLASH detected");
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
  pinMode(AUDIO_PIN, OUTPUT);

  ledcSetup(1, 39000, 10);      // ledchannel, PWM frequency, resolution in bits
  ledcAttachPin(AUDIO_PIN, 1);  // Set Audio pin to timer ch1

  audioSampleTimer = timerBegin(0, 3628, true);             // begins timer 0, 12.5ns*3628 = 45.35usec(22050 Hz), count-up
  timerAttachInterrupt(audioSampleTimer, &audioISR, true);  // edge-triggered
  timerAlarmWrite(audioSampleTimer, 1, true);               // Counts up to 1 then triggers the interrupt, auto-reload
  timerAlarmEnable(audioSampleTimer);                       // enable audioSampleTimer

  Serial.println(F("Audio setup complete"));
}

// These are specific to each IMU, hard code values from callibration programs
void initIMUOffsets() {
  mpu.setXAccelOffset(-3289);
  mpu.setYAccelOffset(-1392);
  mpu.setZAccelOffset(516);
  mpu.setXGyroOffset(240);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(27);
}
void setupIMU() {
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

void setup() {
  Serial.begin(115200);
  setupAudio();
  setupIMU();
  SmoothSwingInit();
  idleState = 1;
  Serial.println("DONE SETUP");
}

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


int16_t ax, ay, az;
int16_t gx, gy, gz;

void loop() {
  if (!dmpReady) return;
  // read a packet from MPU FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    Vec3 gyroVec = Vec3(gyro.x, gyro.y, gyro.z);
    SB_Motion(gyroVec, false);
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  long gyrMag = (long) gx * gx + (long) gy * gy + (long) gz * gz;
  long accMag = (long) ax * ax + (long) ay * ay + (long) az * az;


  if (gyrMag > 45000000 && accMag > 1900000000){
    // Serial.print("gyr: ");
    // Serial.print(gyrMag);
    // Serial.print(" ,accel: ");
    // Serial.println(accMag);
    clash();
  }
  UpdateVolume();
}