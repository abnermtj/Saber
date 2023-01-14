#define SwingStrengthThreshold 80.0f
#define Transition1Degrees 45.0f
#define Transition2Degrees 160.0f
float SwingSensitivity = 2000.0f; // Original 450.0f Lower is more sensistive
#define MaximumHumDucking 75.0f // Orig 75
float SwingSharpness = 1.5f; // Orig 1.75f
float MaxSwingVolume = 3.0f; // Orig 3.0

#include "vec3.h"
#include "box_filter.h"

float lswingVolume = 0;
float hswingVolume = 0;
float humVolume = 1;
bool flip = 0;

uint32_t last_random_ = 0;

BoxFilter<Vec3, 3> gyro_filter_;
uint32_t last_micros_;

enum class SwingState {
  OFF,  // waiting for swing to start
  ON,   // swinging
  OUT,  // Waiting for sound to fade out
};

SwingState state_ = SwingState::OFF;



struct Data {
  void SetTransition(float mp, float w) {
    midpoint = mp;
    width = w;
  }
  float begin() const {
    return midpoint - width / 2;
  }
  float end() const {
    return midpoint + width / 2;
  }
  void rotate(float degrees) {
    midpoint += degrees;
  }
  float midpoint = 0.0;
  float width = 0.0;
};
Data A;
Data B;

void Swap() {
  Data C = A;
  A = B;
  B = C;
  flip = !flip;
}

  // Should only be done when the volume is near zero.
  void PickRandomSwing()
  {
    uint32_t m = millis();
    // No point in picking a new random so soon after picking one.
    if (m - last_random_ < 1000)
      return;
    last_random_ = m;
    

    if (random(2)){ // Either start with low pitch or high pitch
      Swap();
    }

    float t1_offset = random(1000) / 1000.0 * 50 + 10;
    A.SetTransition(t1_offset, 45.0f);         // 10- 60 degrees, 45.0f
    B.SetTransition(t1_offset + 180.0, 160.0f); // 190 - 240degrees, 160f default
  }

  void SB_Init(){
    PickRandomSwing();
  }
void SB_Motion(const Vec3 &raw_gyro, bool clear) {
  if (clear) {
    gyro_filter_.filter(raw_gyro);
    gyro_filter_.filter(raw_gyro);
  }
  Vec3 gyro = gyro_filter_.filter(raw_gyro);
  // degrees per second
  // May not need to smooth gyro since volume is smoothed.
  
  
  float speed = sqrtf(gyro.z * gyro.z + gyro.x * gyro.x);
  uint32_t t = micros();
  uint32_t delta = t - last_micros_;
  if (delta > 1000000)
    delta = 1;
  last_micros_ = t;
  float hum_volume = 1.0;

  switch (state_) {
    case SwingState::OFF:
      if (speed < SwingStrengthThreshold) {
        // Serial.print("speed: ");
        // Serial.println(speed);
        break;
      }
      state_ = SwingState::ON;

    case SwingState::ON:
      if (speed >= SwingStrengthThreshold * 0.9) {
        float swing_strength = min(1.0f, speed / SwingSensitivity);

        // 50000000 was a good value below
        A.rotate(-speed * delta / 80000000.0);  // This shifts the midpoint of the swing, it accumulates.
     // A.rotate(-speed * delta / 1000000.0); // Th
        // original value is 1000000.0
        // Reache the midpoint of the swing here
        // If the current transition is done, switch A & B,
        // and set the next transition to be 180 degrees from the one
        // that is done.
        while (A.end() < 0.0) {
          Serial.println("TRANSITION");
          B.midpoint = A.midpoint + 180.0;
          Swap();
        }
        float mixab = 0.0;
        if (A.begin() < 0.0)
          mixab = constrain(-A.begin() / A.width, 0.0, 1.0);

        float mixhum = pow(swing_strength, SwingSharpness); 

        hum_volume = 1.0 - mixhum * MaximumHumDucking / 100.0;  // OUPUT

        mixhum *= MaxSwingVolume;  // OUTPUT

        
          // Serial.print("speed: ");
          // Serial.print(speed);
          // Serial.print(" \tR: ");
          // Serial.print(-speed * delta / 1000000.0);
          // Serial.print(" \tMP: ");
          // Serial.print(A.midpoint);
          // Serial.print(" \tB: ");
          // Serial.print(A.begin());
          // Serial.print(" \tE: ");
          // Serial.print(A.end());
          Serial.print(" \t,lvol: ");
          Serial.print(lswingVolume);
          Serial.print(" \t,hvol: ");
          Serial.print(hswingVolume);
          Serial.print(" \t,mixhum: ");
          Serial.print(mixhum);
          Serial.print(" \t,mixab: ");
          Serial.print(mixab);
          Serial.print(" \t,hum_volume: ");
          Serial.println(hum_volume);
 

          if (flip) {
            lswingVolume = mixhum * mixab;
            hswingVolume = mixhum * (1.0 - mixab);
          } else {
            hswingVolume = mixhum * mixab;
            lswingVolume = mixhum * (1.0 - mixab);
          }

        
        break;
      }
      lswingVolume = 0;
      hswingVolume = 0;

      state_ = SwingState::OUT;

    case SwingState::OUT:
      PickRandomSwing();
      state_ = SwingState::OFF;
  }
  // Must always set hum volume, or fade-out doesn't work.
  humVolume = hum_volume;
}