
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif


// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(144, LED_PIN, NEO_GRB + NEO_KHZ800);

void setupLED()
{
    strip.begin();
    strip.setBrightness(150);
    strip.show(); // Initialize all pixels to 'off'
}

void breatheLED(float MaximumBrightness, float MinimumBrightness, float SpeedFactor, float StepDelay) // TODO fix this neeeds to run concurrently with other operations
{
    // Make the lights breathe
    for (int i = 0; i < 65535; i++)
    {
        // Intensity will go from 10 - MaximumBrightness in a "breathing" manner
        float intensity = MaximumBrightness / 2.0 * (1.0 + sin(SpeedFactor * i)) + MinimumBrightness;
        intensity = constrain(intensity, 0, 255);
        strip.setBrightness(intensity);
        Serial.print("Intensity: ");
        Serial.println(intensity);
        // Now set every LED to that color
        for (int ledNumber = 0; ledNumber <= 144; ledNumber++)
        {
            strip.setPixelColor(ledNumber, 255, 0, 0);
        }

        strip.show();
        // Wait a bit before continuing to breathe
        delay(StepDelay);
    }
}

// TODO Clean up
void IgniteLED(uint32_t color, uint8_t duration, uint32_t tipcolor, uint8_t transitiontype, uint32_t transitionduration)
{
  Serial.println("igniting LEDS");
    uint16_t j, led;
    (led = 0);
    static bool ongoing = true;
    while (ongoing)
    {
        for (j = 0; j <= 147; j++)
        { // 147 is 144 + 6 which is the below j>6
            if (j <= 3)
                ;
            strip.setPixelColor(j, tipcolor);
            strip.show();
            delay(duration);
            if (j > 3)
            {
                strip.setPixelColor(led, color);
                strip.show();
                delay(duration);
                led++;
            }
        }
        ongoing = false;
        strip.setPixelColor(led, color);
        strip.show();
        delay(duration);
    }
}

void RetractLED(uint8_t duration)
{
    for (uint16_t pixelposition = 0; pixelposition < strip.numPixels(); pixelposition++)
    {
        strip.setPixelColor(144 - pixelposition, 0);
        strip.show();
        delay(duration);
    }
}
