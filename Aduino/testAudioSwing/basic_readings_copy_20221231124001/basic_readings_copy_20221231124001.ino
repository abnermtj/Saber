// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MPU6050_6Axis_MotionApps20.h>
#include "audio.h"
#include "Arduino.h"
MPU6050 mpu;
// orientation/motion vars
Quaternion curRotation;             // [w, x, y, z]         quaternion container
Quaternion prevRotation;            // [w, x, y, z]         quaternion container
static Quaternion prevOrientation;  // [w, x, y, z]         quaternion container
static Quaternion curOrientation;   // [w, x, y, z]         quaternion container
VectorInt16 curAccel;
VectorInt16 prevAccel;
VectorInt16 curDeltAccel;
VectorInt16 prevDeltAccel;

#define CLASH_THRESHOLD 10
volatile bool mpuInterruptDetected = false;  // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;                        // holds actual interrupt status byte from MPU
uint16_t DMPpacketSize;                      // expected DMP packet size (default is 42 bytes)
uint8_t mpuFifoBuffer[64];                   // FIFO storage buffer
uint16_t mpuFifoCount;                       // count of all bytes currently in FIFO
#define MPU_INTERRUPT_PIN 2
bool dmpReady = false;
I2Cdev i2ccomm;


inline void dmpDataReady() {
  mpuInterruptDetected = true;
}  //dmpDataReady

void ISR_MPUInterrupt() {
  Serial.println("CLASH");
  if (false) {  // TODO SABER CLASH HERE
    // FX_Clash();
  }
}
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens


  // MPU6050_start();
   Serial.println("SKIPPED MPU");
  // start sound system
  snd_init();
}

int ctrl_counter = 0;
byte global_volume = 240;
int snd_buzz_base_speed = 47;
int snd_hum1_base_speed = 50;
int snd_hum2_base_speed = 52;
int snd_hum2_doppler = 40;
int snd_echo_decay = 128;
unsigned int entropy = 0;
void add_entropy(byte e, byte mask) {
  entropy = entropy << 1 ^ (e & mask);
}

int accel[3];
int accel_last[3];
int gyro[3];

float velocity[3];
float recent_impulse = 0;

float rotation_history = 0.0;
float rotation_offset = 0.0;
float rotation_factor = 0.0;

float rotation_echo = 0.0;

float velocity_offset = 26.6;
float velocity_factor = 0;

int gyro_hum1_volume = 0;
int accel_hum1_volume = 0;


VectorInt16 curGyro;




float vec3_length(float *v) {
  float r = 0;
  for (int i = 0; i < 3; i++) r += (v[i] * v[i]);
  return sqrt(r);
}

void vec3_addint(float *v, int *a) {
  for (int i = 0; i < 3; i++) v[i] += a[i];
}

void vec3_scale(float *v, float n) {
  for (int i = 0; i < 3; i++) v[i] *= n;
}

// add a delta to a value, and limit the result to a range
int value_delta(int value, int delta, int vmin, int vmax) {
  int rd = 0;
  if (delta > 0) {
    rd = min(vmax - value, delta);
  }
  if (delta < 0) {
    rd = max(vmin - value, delta);
  }
  return value + rd;
}


void loop() {
  // motionEngine();

  int i, n, delta;
  float av, rv;
  // the program is alive
  //wdt_reset();
  // alternately read from the gyro and accelerometer
  ctrl_counter = ctrl_counter ^ 1;
  if ((ctrl_counter & 1) == 1) {
    // sample gyro
    MPU6050_gyro_vector(gyro);
    add_entropy(gyro[0], 0x0F);
    // int3_print(gyro);
    // rotation vector, made from only two axis components (ignore 'twist')
    float gv[3];
    gv[0] = gyro[0];
    gv[1] = gyro[2];
    gv[2] = 0.0;

    // vector length
    float rot = vec3_length(gv);
    rotation_offset -= (rotation_offset - rot) / 300.0;
    // rotation_history = rotation_history * 0.95 + (rot - rotation_offset) / 1000.0;
    rv = (rot - rotation_offset) / 50.0;
    rv = (rotation_history + rv) / 2.0;
    rotation_history = rv;
    //  Serial.println(rv);
  } else {
    MPU6050_accel_vector(accel);
    add_entropy(accel[0], 0x0F);
    // update the velocity vector

    vec3_addint(velocity, accel);
    vec3_scale(velocity, 0.99);
    // turn velocity vector into scalar factor
    av = vec3_length(velocity) / 10000.0;
    velocity_offset -= (velocity_offset - av) / 10.0;
    velocity_factor = sqrt(abs(velocity_offset - av));
    if (velocity_factor > 1.0) velocity_factor = 1.0;
  }

  // check knobv
  n = analogRead(A0);
  add_entropy(n, 0x03);

  // use some entropy to modulate the sound volume
  snd_buzz_speed = snd_buzz_base_speed + (entropy & 3);
  snd_hum1_speed = snd_hum1_base_speed;

  // rotation hum and pitch-bend
  rv = rotation_history;
  if (rv < 0.0) rv = 0.0;
  if (rv > 140.0) rv = 120.0;
  // update the rotation echo
  if (rv > rotation_echo) {
    // the echo is maximised
    rotation_echo = rv;
  } else {
    // decay the previous echo
    rotation_echo = rotation_echo * (0.975f + (float)snd_echo_decay / 10240.0f);
    // use the louder of the original value and 1/1.6 the echo
    rv = max(rv, rotation_echo / 1.6);
  }
  // rotation volume term
  rv = rv / 256.0 * global_volume;
  delta = 0;
  if (rv > snd_hum2_volume) {
    delta = 16;
  } else if (rv < snd_hum2_volume) {
    delta = -8;
  }
  snd_hum2_volume = value_delta(snd_hum2_volume, delta, 0, 255);

  // Serial.println(rotation_history);
  snd_hum1_speed = snd_hum1_base_speed + (rotation_history / snd_hum2_doppler);
  snd_hum2_speed = snd_hum2_base_speed + (rotation_history / snd_hum2_doppler);


  // Serial.print(rotation_history);
  // Serial.print(" ");
  // Serial.print(snd_hum1_speed);
  // Serial.print(" ");
  // Serial.print(rotation_history);
  // Serial.print(" ");
  // Serial.println(rotation_history);
  // Serial.print(snd_buzz_speed);
  // Serial.print(" ");
  // Serial.print(snd_hum1_speed);
  // Serial.print(" ");
  // Serial.print(snd_hum2_speed);
  // Serial.print(" ");
  // Serial.print(snd_buzz_volume);
  // Serial.print(" ");
  // Serial.print(snd_hum1_volume);
  // Serial.print(" ");
  // Serial.println(snd_hum2_volume);
  // turn velocity into volume modifications


  av = velocity_factor;
  if (av > 1.0) av = 1.0;
  snd_buzz_volume = 8 + (int)(av * 32.0);
  snd_buzz_volume = ((unsigned int)snd_buzz_volume * (unsigned int)global_volume) / 256;
  snd_hum1_volume = 12 + (int)(av * 40.0);
  snd_hum1_volume = max(((unsigned int)snd_hum1_volume * (unsigned int)global_volume) / 256, snd_hum2_volume);

  //// DEBUG ///////////////////////////////////////////////////////////////
//   unsigned int v1 = snd_buzz_volume;
//    v1 *= 10;  //v1 = v1 >> 8;
//   unsigned int v2 = snd_hum1_volume;
//    v2 *= 10;  //v2 = v2 >> 8;
//   unsigned int v3 = snd_hum2_volume;
//    v3 *= 10;  //v3 = v3 >> 8;
//   // sample our primary waveforms, and multiply by their master volumes
//   int s1 = (sound_sample(&snd_index_1, buzz_wave, snd_buzz_speed, BUZZ_WAVE_LENGTH) - 128) * v1;
//   int s2 = (sound_sample(&snd_index_2, hum1_wave, snd_hum1_speed, HUM1_WAVE_LENGTH) - 128) * v2;
//   int s3 = (sound_sample(&snd_index_3, hum2_wave, snd_hum2_speed, HUM2_WAVE_LENGTH) - 128) * v3;
//  unsigned int sample = 0x8000 + s1 + s2 + s3;

//   int test  = (sample >> 9) ;
  // //sample = ((sample >> 8) & 0xff -120) *2+ 100 ;

//   // Serial.print("HERE2\t ");
//   // Serial.print(s1);
//   // Serial.print("\t");
//   // Serial.print(s2);
//   // Serial.print("\t");
//   // Serial.print(s3);
//   // Serial.print("\t");

    // Serial.println(test);
  //// DEBUG ///////////////////////////////////////////////////////////////
  // active
  // Serial.print(velocity_factor); Serial.print(' ');
  // Serial.println(rotation_history);
}

inline void printQuaternion(Quaternion quaternion) {
  if (quaternion.w * 1000 < 999.6 || abs(quaternion.x + quaternion.y + quaternion.z) > 0.3) {
    Serial.print(F("\t\tQ\t\tw= "));
    Serial.print(quaternion.w * 1000);
    Serial.print(F("\t\tsum= "));
    Serial.println(abs(quaternion.x + quaternion.y + quaternion.z));
    // Serial.print(F("\t\tx= "));
    // Serial.print(quaternion.x);
    // Serial.print(F("\t\ty= "));
    // Serial.print(quaternion.y);
    // Serial.print(F("\t\tz= "));
    // Serial.println(quaternion.z);
  }
}  //printQuaternion

inline void motionEngine() {
  if (!dmpReady)
    return;

  mpuInterruptDetected = false;
  mpuIntStatus = mpu.getIntStatus();  // INT_STATUS byte
  mpuFifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    curDeltAccel.x = 0;
    curDeltAccel.y = 0;
    curDeltAccel.z = 0;
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (mpuFifoCount < DMPpacketSize)
      mpuFifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(mpuFifoBuffer, DMPpacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    mpuFifoCount -= DMPpacketSize;

    prevOrientation = curOrientation.getConjugate();
    prevAccel = curAccel;

    mpu.dmpGetQuaternion(&curOrientation, mpuFifoBuffer);

    mpu.dmpGetAccel(&curAccel, mpuFifoBuffer);
    curDeltAccel.x = prevAccel.x - curAccel.x;
    curDeltAccel.y = prevAccel.y - curAccel.y;
    curDeltAccel.z = prevAccel.z - curAccel.z;

    mpu.dmpGetGyro(&curGyro, mpuFifoBuffer);




    // Serial.print("G.x ");
    // Serial.print(curGyro.x);

    // Serial.print(" G.y ");
    // Serial.print(curGyro.y);
    // Serial.print(" G.z ");
    // Serial.println(curGyro.z);

    // Serial.print("curDeltAccel.x ");
    // Serial.print(curDeltAccel.x);

    // Serial.print(" curDeltAccel.y ");
    // Serial.print(curDeltAccel.y);
    // Serial.print(" curDeltAccel.z ");
    // Serial.println(curDeltAccel.z);

    //We calculate the rotation quaternion since last orientation
    prevRotation = curRotation;
    curRotation = prevOrientation.getProduct(
      curOrientation.getNormalized());
    // display quaternion values in easy matrix form: w x y z
    // printQuaternion(curRotation);
  }
}




// i2c utility functions
bool write_packet(byte device, int address, int data) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
  return true;
}

bool read_packet(byte device, byte address, byte *buffer, byte length) {
  // send phase
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
  // recieve phase
  Wire.beginTransmission(device);
  Wire.requestFrom(device, (uint8_t)length);
  // how many did we get?
  byte r = Wire.available();
  if (r <= length) {
    for (byte i = 0; i < r; i++) {
      buffer[i] = Wire.read();
      // Serial.print("."); Serial.print(buffer[i]);
    }
  } else {
    // consume unexpected bytes
    for (byte i = 0; i < length; i++) { Wire.read(); }  //  { Serial.print("!"); Serial.print(Wire.read()); }
  }
  Wire.endTransmission();
  // if(r < length) { Serial.print(" packet too short! "); Serial.print(r); }
  // if(r > length) { Serial.print(" packet too long! "); Serial.print(r); }
  return (r == length);
}

/*
 * MPU6040 Accellerometer + Gyro
 * 
 */
static const byte I2C_MPU6050 = 0x68;  // i2c device address
// public properties
void MPU6050_start() {
  // reset
  write_packet(I2C_MPU6050, 0x6B, 0x80);  //
  delay(50);
  // configuration
  write_packet(I2C_MPU6050, 0x19, 0x01);  // Sample Rate
  write_packet(I2C_MPU6050, 0x6B, 0x03);  // Z-Axis gyro reference used for improved stability (X or Y is fine too)
  delay(30);
  write_packet(I2C_MPU6050, 0x1B, 0x18);  // Gyro Configuration FS_SEL = 3
  delay(30);
  write_packet(I2C_MPU6050, 0x1C, 0b11101000);  // Accel Configuration AFS_SEL = 1
  delay(20);
}

void MPU6050_stop() {
}

bool MPU6050_get_vector(byte reg, int *v) {
  byte buffer[6];
  if (read_packet(I2C_MPU6050, reg, buffer, 6)) {
    // [todo: bit shuffling might not be necessary - try just dumping directly into the vector]
    v[0] = (buffer[0] << 8) | buffer[1];
    v[1] = (buffer[2] << 8) | buffer[3];
    v[2] = (buffer[4] << 8) | buffer[5];
    return true;
  } else {
    return false;
  }
}

bool MPU6050_gyro_vector(int *v) {
  return MPU6050_get_vector(0x43, v);
}

bool MPU6050_accel_vector(int *v) {
  return MPU6050_get_vector(0x3B, v);
}

bool MPU6050_temp_vector(int *v) {
  byte buffer[2];
  if (read_packet(I2C_MPU6050, 0x41, buffer, 2)) {
    // [todo: bit shuffling might not be necessary - try just dumping directly into the vector]
    v[0] = (buffer[0] << 8) | buffer[1];
    return true;
  } else {
    return false;
  }
}

bool MPU6050_get_ident() {
  byte buffer[3];
  // [todo: might not be necessary - try just dumping directly into the vector]
  return read_packet(I2C_MPU6050, 0x75, buffer, 1) && (buffer[0] == 0x68);
}



void int3_add(int *v, int *a) {
  for (int i = 0; i < 3; i++) v[i] += a[i];
}

void int3_sub(int *v, int *a) {
  for (int i = 0; i < 3; i++) v[i] -= a[i];
}

void int3_print(int *v) {
  for (int i = 0; i < 3; i++) {
    Serial.print(v[i]);
    Serial.print(' ');
  }
  Serial.println();
}
void vec3_print(float *v) {
  for (int i = 0; i < 3; i++) {
    Serial.print(v[i]);
    Serial.print(' ');
  }
  Serial.println();
}