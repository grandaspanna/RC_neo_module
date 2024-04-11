# RC_neo_module
Compact module for controlling standard and smart (WS2812B etc) LEDs in an RC vehicle

The module is based on a Microchip attiny1616 microcontroller and features:
* Two PWM inputs
* 4 MOSFET-controlled 6V ouputs
* Two neopixel outputs and a 5V 1A supply capability

The software is written in the Arduino IDE and leverages the MegaTinyCore (https://github.com/SpenceKonde/megaTinyCore) by Spencer Konde.
The code uses the built-in features of the microcontroller to precisely measure the PWM signal width and even at 4MHz clock rate, has a better than 1 microsecond resolution.

In summary, the module is designed to make sophisticated lighting patterns possible in response to the two PWM signals being monitored. They can be any two (or one) channel from an RC receiver. The use of channel mirroring/mixing or direct tapping into the throttle or steering channels via a y-cable is possible and practical.
This is an example of the module in use: https://www.youtube.com/watch?v=tlSBbWml3F8

# Output pins
The preferred pin naming is used in the code. The two pins attached to the neopixel outputs are:
* PIN_PB3
* PIN_PA4

The four MOSFET controlled pins are:
* PIN_PC1
* PIN_PC2
* PIN_PC0
* PIN_PC3

# Acquisition of PWM values
The software makes use of the two built-in TCB timers to accurately capture the pulse width. They are connected to the two input pins:
* PIN_PA5
* PIN_PB5

For all intents and purposes, it is probably best to treat that part of the code as "black box". It leverages the event subsystem of the microcontroller and the timers to automatically capture the values and it does this with incredible accuracy and low overhead. The example code captures the current values in global variables that are updated by interrupts in the background. There are two types:
* PWM pulse width (pwm1_pulse_width and pwm2_pulse_width) which is the number of TCB clock ticks
* PWM value (pwm1_value & pwm2_value) which is the pulse width in microseconds

Be wary of changing code or running different clock frequencies, as the code is mostly written around a 4MHz clock frequency which is both robust and accurate. It yields 0.5 microsecond resolution as-is.
