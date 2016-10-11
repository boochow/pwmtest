# pwmtest
PWM sound synthesize test for Arduboy

You also need [Arduboy2 library](https://github.com/MLXXXp/Arduboy2).

Synth engine is ported from Kamide's [PWMDAC_Synth](https://ja.osdn.net/users/kamide/pf/PWMDAC_Synth/wiki/FrontPage), modified for ATmega32u4.

Arduboy has two PWM pins, pin 5 and pin 13.

In your sketch, use
```
#define PWMDAC_OUTPUT_PIN 13
#define PWMDAC_OUTPUT_PIN2 5
```
for larger sound. You can also use
```
#define PWMDAC_OUTPUT_PIN 5
#define PWMDAC_OUTPUT_PIN2 13
```
which will generate more quiet sound.

YouTube video is [here](https://www.youtube.com/watch?v=aCzUDXMH9O8).
