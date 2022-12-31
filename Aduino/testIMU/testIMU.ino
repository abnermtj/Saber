#include "MPU9250.h"

MPU9250 mpu;

void setup(void) {
  Serial.begin(115200);

  Wire.begin();
  delay(2000);        // Wait for mpu to initialize
  mpu.verbose(true);  // For debug
  mpu.setup(0x68);    // Set to i2c address of mpu
}

void loop(void) {
  if (mpu.update()) {
    
    String url = "accX:";
    url.concat(mpu.getAccX());
    url.concat(", accY:");
    url.concat(mpu.getAccY());
    url.concat(", accZ:");
    url.concat(mpu.getAccZ());
    url.concat(", GyrX:");
    url.concat(mpu.getGyroX());
    url.concat(", GyrY:");
    url.concat(mpu.getGyroY());
    url.concat(", GyrZ:");
    url.concat(mpu.getGyroZ());
    //url.concat(" ");
    //url.concat(millis());
    Serial.println(url);
  }

  delay(50);  // 20Hz
}
