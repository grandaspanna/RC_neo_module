# RC_neo_module
Compact module for controlling standard and smart (WS2812B etc) LEDs in an RC vehicle

The module is based on a Microchip attiny1616 microcontroller and features:
* Two PWM inputs
* 4 MOSFET-controlled 6V ouputs
* Two neopixel outputs and a 5V 1A supply capability

The software is written in the Arduino IDE and leverages the MegaTinyCore (https://github.com/SpenceKonde/megaTinyCore) by Spencer Konde.
The code uses the built-in features of the microcontroller to precisely measure the PWM signal width and even at 4MHz clock rate, has a better than 1 microsecond resolution.

In summary, the module is designed to make sophisticated lighting patterns possible in response to the two PWM signals being monitored. They can be any two (or one) channel from an RC receiver. The use of channel mirroring/mixing or direct tapping into the throttle or steering channels via a y-cable is possible and practical.
