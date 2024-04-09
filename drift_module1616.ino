// 
// Code to support module that reads two RC PWM signals and switches some MOSFETS
// or 2 neopixel channels in response
// Uses attiny1616 microcontroller via the megatinycore add-on core for Arduino
// Code currently assumes 4MHz clock rate to map pulse width counts to microseconds
//
// uses both TCB timers in "pulse width mode" and via interrupts, maintains a global variable
// that holds the last read PWM value. These values are updated every time a new cycle is read
// automatically by the microcontroller
//
// The use of analogWrite for controlling the brightness of LEDs switched by the MOSFETs
// is supported, but pay attention to which timers are used for analog output
//

#include "flasher.h"

// Declarations for the neopixel outputs
#include <tinyNeoPixel.h>
#define NEO_PIN1 PIN_PB3
#define NEO_PIN2 PIN_PA4

#define WING_PIN NEO_PIN1
#define WING_PIXELS 26
#define BUMPER_PIN NEO_PIN2
#define BUMPER_PIXELS 5
tinyNeoPixel wingStrip = tinyNeoPixel(WING_PIXELS, WING_PIN, NEO_GRB + NEO_KHZ800);
tinyNeoPixel bumperStrip = tinyNeoPixel(BUMPER_PIXELS, BUMPER_PIN, NEO_GRB + NEO_KHZ800);
#define DEFAULT_WING_BLINK_INTERVAL 500
int wingBlinkInterval;
uint16_t lastWingBlinkUpdate;
boolean wingBlinkOn = true;


#define NEUTRAL 0
#define ACCELERATING 1
#define BRAKING 2
#define REAR_UPDATE_TIME 75
uint16_t lastRearUpdateTime;

// Declarations for the four MOSFET-controlled outputs
#define OUTPUT_A PIN_PC1
#define OUTPUT_B PIN_PC2
#define OUTPUT_C PIN_PC0
#define OUTPUT_D PIN_PC3

// Declarations for the two PWM inputs
#define PWM_INPUT1 PIN_PA5
#define PWM_INPUT2 PIN_PB5

Flasher brakeLights(OUTPUT_C,100,250);
Flasher leftLights(OUTPUT_A,500,500);
Flasher rightLights(OUTPUT_B,500,500);


unsigned long current_millis;
uint16_t  last_time;
volatile unsigned int pwm1_pulse_width, pwm2_pulse_width;
volatile uint16_t pwm1_value = 0, pwm2_value;
volatile int prev_time = 0;
boolean signal_received = false;


unsigned long threshold_accel, threshold_brake;

void setup() {
  // Short delay on startup
  delay(500);

  // Setup the two PWM pins as input with pullups
  pinMode(PWM_INPUT1,INPUT_PULLUP);
  pinMode(PWM_INPUT2,INPUT_PULLUP);

  // Setup the 4 outputs
  pinMode(OUTPUT_A,OUTPUT);
  pinMode(OUTPUT_B,OUTPUT);
  pinMode(OUTPUT_C,OUTPUT);
  pinMode(OUTPUT_D,OUTPUT);

  // Setup the first neopixel strip
  wingStrip.begin();
  wingStrip.clear();
  wingStrip.show();
  // Setup the second neopixel strip
  bumperStrip.begin();
  bumperStrip.clear();
  bumperStrip.show();
  wingBlinkInterval = DEFAULT_WING_BLINK_INTERVAL;
  lastWingBlinkUpdate = millis();
 
  // Just show the outputs working - blink twice
  for(int i=0;i<2;i++){
    digitalWriteFast(OUTPUT_A,HIGH);
    delay(50);
    digitalWriteFast(OUTPUT_A,LOW);
    digitalWriteFast(OUTPUT_B,HIGH);
    delay(50);
    digitalWriteFast(OUTPUT_B,LOW);
    digitalWriteFast(OUTPUT_C,HIGH);
    delay(50);
    digitalWriteFast(OUTPUT_C,LOW);
    digitalWriteFast(OUTPUT_D,HIGH);
    delay(50);
    digitalWriteFast(OUTPUT_D,LOW);
  }
  
  // The hear of the PWM input measurement is in the two TCB timers
  // Setup TCB0 & TCB1 to pulse width measurement mode
  TCB_0_init();
  TCB_1_init();
  
  // Connect the input pins to the event system so they get processed by the
  // relevant TCB timers
  // Set up event system to process inbound activity on the pins
  EVENT_SYSTEM_init();

  // Wait until getting sensible readings
  // Code below assumes PWM value falls between 1000 and 2000. Note that some radios have narrower PWM ranges.
  boolean ready = false;
  int try_count = 0;
  while(!ready && (try_count++ < 20) ){
    if(pwm1_value > 1000 && pwm1_value < 2000){
      ready = true;
    }
    delay(40);
  }

  if(ready){
    // Something happened, blink twice    
    for(int i=0;i<2;i++){
      PORTA.OUTSET = PIN2_bm;
      delay(50);
      PORTA.OUTCLR = PIN2_bm;
      delay(50);
    }
      
    // Grab an average of ten readings, a small time apart
    unsigned long totes =0;
    for(int i=0;i<10;i++){
      totes += pwm1_value;
      delay(50);
    }
    totes = totes/10;
    threshold_accel = totes * 1.025;
    threshold_brake = totes * 0.975;

    // Mark signal good
    signal_received = true;
  }
  else{
    // We didn't get a sensible value
    // Blink 5 times
    for(int i=0;i<6;i++){
      PORTC.OUTSET = PIN2_bm;
      delay(70);
      PORTC.OUTCLR = PIN2_bm;
      delay(70);
    }
   
  }

   //analogWrite(OUTPUT_D,64); //Low brightness on headlight port

  // Setup TCA0 to millisecond interrupt mode (compare mode)
  // This should generate an interrupt every millisecond or so
  //setupTCA0_4();
  //setupTCA0_20();
  
  // End of setting stuff up, as opposed to stuffups, but you never know...
}


void loop() {
  
  uint16_t esc_input;
  uint32_t bumperColour,orange,red,green,wingBlinkColour;
  int mode;
  
  if(signal_received)
    esc_input = pwm1_value;
  else
    esc_input = 0;

  // Assume neutral unless otherwise told (if no signal,just do braking pattern)
  mode = signal_received ? NEUTRAL : BRAKING;
  
  if(esc_input > threshold_accel)
    mode = ACCELERATING;
  if(esc_input < threshold_brake)
    mode = BRAKING;

  red = wingStrip.Color(255,0,0);
  green = wingStrip.Color(0,255,0);
  orange = wingStrip.Color(255,80,0);

  switch(mode){
    case NEUTRAL: //digitalWriteFast(OUTPUT_A,LOW);
                  //digitalWriteFast(OUTPUT_B,LOW);
                  leftLights.dutyCycle(50,500);
                  rightLights.dutyCycle(50,500);
                  bumperColour = 0;
                  wingBlinkInterval = DEFAULT_WING_BLINK_INTERVAL;
                  break;;
    case ACCELERATING: //digitalWriteFast(OUTPUT_A,HIGH);
                  //digitalWriteFast(OUTPUT_B,LOW);
                  leftLights.dutyCycle(10,700);
                  rightLights.dutyCycle(10,700);
                  bumperColour = green;
                  wingBlinkInterval = DEFAULT_WING_BLINK_INTERVAL * 7 / 10;
                  break;;
    case BRAKING: //digitalWriteFast(OUTPUT_A,LOW);
                  //digitalWriteFast(OUTPUT_B,HIGH);
                  leftLights.dutyCycle(60,80);
                  rightLights.dutyCycle(60,80);
                  bumperColour = red;
                  wingBlinkInterval = DEFAULT_WING_BLINK_INTERVAL * 4 / 10;
                  break;;
  }

  current_millis = millis();
  leftLights.Update(current_millis);
  rightLights.Update(current_millis);
  // Turn on channel D as inverse of channel A
  digitalWriteFast(OUTPUT_D,!digitalReadFast(OUTPUT_A));

  // Decide state of outer parts of wing light
  if((millis() - lastWingBlinkUpdate) > wingBlinkInterval){
    lastWingBlinkUpdate = millis();
    // toggle the outer blink
    wingBlinkOn = !wingBlinkOn;
    if(wingBlinkOn)
      wingBlinkColour = orange;
    else
      wingBlinkColour = 0;
  }
  // Do the rear bars
  
  if((millis() - lastRearUpdateTime) > REAR_UPDATE_TIME){
    lastRearUpdateTime = millis();
    // Update bumper
    for(int i=0; i < bumperStrip.numPixels();i++){
      bumperStrip.setPixelColor(i,bumperColour);
    }
    for(int i=0; i < wingStrip.numPixels(); i++ ){
      wingStrip.setPixelColor(i,bumperColour);
    }
    // Outer ones are orange
    for(int i=0 ; i < 4 ; i++){
      wingStrip.setPixelColor(i,wingBlinkColour);
      int lastPixel = wingStrip.numPixels() - 1;
      wingStrip.setPixelColor(lastPixel - i,wingBlinkColour);
    }
    bumperStrip.show();
    wingStrip.show();
  }
  //delay(10);
}

int8_t TCB_0_init()
{

  // This code sets up TCB0 in pulse width capture mode
  // Code below configures system clock divided by 2
  
  TCB0.CTRLB = 0 << TCB_ASYNC_bp      /* Asynchronous Enable: disabled */
               | 0 << TCB_CCMPEN_bp   /* Pin Output Enable: disabled */
               | 0 << TCB_CCMPINIT_bp /* Pin Initial State: disabled */
               | TCB_CNTMODE_PW_gc;   /* Input Capture Pulse-Width measurement */

  TCB0.EVCTRL = 1 << TCB_CAPTEI_bp    /* Event Input Enable: enabled */
                | 0 << TCB_EDGE_bp    /* Event Edge: disabled */
                | 0 << TCB_FILTER_bp; /* Input Capture Noise Cancellation Filter: disabled */

  TCB0.INTCTRL = 1 << TCB_CAPT_bp;   /* Capture or Timeout: enabled */
     

  TCB0.CTRLA = TCB_CLKSEL_DIV2_gc     /* CLK_PER/2 (From Prescaler) */
               | 1 << TCB_ENABLE_bp   /* Enable: disabled */
               | 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
               | 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */

  return 0;
}


int8_t TCB_1_init()
{

  // This code sets up TCB1 in pulse width capture mode
  // Code below configures system clock divided by 2

  TCB1.CTRLB = 0 << TCB_ASYNC_bp      /* Asynchronous Enable: disabled */
               | 0 << TCB_CCMPEN_bp   /* Pin Output Enable: disabled */
               | 0 << TCB_CCMPINIT_bp /* Pin Initial State: disabled */
               | TCB_CNTMODE_PW_gc;   /* Input Capture Pulse-Width measurement */

  TCB1.EVCTRL = 1 << TCB_CAPTEI_bp    /* Event Input Enable: enabled */
                | 0 << TCB_EDGE_bp    /* Event Edge: disabled */
                | 0 << TCB_FILTER_bp; /* Input Capture Noise Cancellation Filter: disabled */

  TCB1.INTCTRL = 1 << TCB_CAPT_bp /* Capture or Timeout: enabled */;

  TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc  /* CLK_PER/2 (From Prescaler) */
               | 1 << TCB_ENABLE_bp   /* Enable: enabled */
               | 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
               | 0 << TCB_SYNCUPD_bp; /* Synchronize Update: disabled */

  return 0;
}

// This interrupt runs when the pulse width measurement is triggered (falling edge after rising edge)
ISR(TCB0_INT_vect)
{
  // This interrupt is triggered whenever a pulse width capture event is triggered
  // by the TCB0 timer, so we capture the value and reset the interrupt.
  
  pwm1_pulse_width = TCB0.CCMP;  // Count of clock cycles
  //pwm_value = pulse_width/10; // 20MHz, div2 prescaler means 10 counts per uSec
  pwm1_value = pwm1_pulse_width/2; // 4MHz, div2 prescaler means 2 counts per uSec

  //PORTC.OUTSET = PIN0_bm;
  /**
   * The interrupt flag is cleared by writing 1 to it, or when the Capture register
   * is read in Capture mode
   */
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect)
{
  // This interrupt is triggered whenever a pulse width capture event is triggered
  // by the TCB1 timer, so we capture the value and reset the interrupt.
  
  pwm2_pulse_width = TCB1.CCMP;  // Count of clock cycles
  //pwm_value = pulse_width/10; // 20MHz, div2 prescaler means 10 counts per uSec
  pwm2_value = pwm2_pulse_width/2; // 4MHz, div2 prescaler means 2 counts per uSec

  /**
   * The interrupt flag is cleared by writing 1 to it, or when the Capture register
   * is read in Capture mode
   */
  TCB1.INTFLAGS = TCB_CAPT_bm;
}

// Setup the event system. Connect the input pins to the event channels.
int8_t EVENT_SYSTEM_init()
{

  EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_SYNCCH0_gc; /* Synchronous Event Channel 0 */

  EVSYS.ASYNCUSER11 = EVSYS_ASYNCUSER11_SYNCCH1_gc; /* Synchronous Event Channel 1 */

  EVSYS.SYNCCH0 = EVSYS_SYNCCH0_PORTA_PIN5_gc; /* Synchronous Event from Pin PA5 */

  EVSYS.SYNCCH1 = EVSYS_SYNCCH1_PORTB_PIN5_gc; /* Synchronous Event from Pin PB5 */
 
  return 0;
}
