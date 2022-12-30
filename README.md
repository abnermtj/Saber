REPLACE
DFPlayer - A Mini MP3 Player For Arduino
1.0.5

https://github.com/DFRobot/DFRobotDFPlayerMini


Processor (old bootloader in settings)

adafruit neopixel 1.10.7



1. Install https://github.com/hideakitai/MPU9250

    For the library above, need to change values in MPU9250.h:
    Windows Location: C:\Users\USER NAME\Documents\Arduino\libraries\MPU9250\MPU9250.h
    ```
    static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE{0x00}; // Originally 0x48
    static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE{0x75}; // Originally 0x71