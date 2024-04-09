class Flasher
{
  // Class Member Variables
  // These are initialized at startup
  int ledPin;      // the number of the LED pin
  long OnTime;     // milliseconds of on-time
  long OffTime;    // milliseconds of off-time
 
  // These maintain the current state
  int ledState;                 // ledState used to set the LED
  unsigned long previousMillis;   // will store last time LED was updated
 
  // Constructor - creates a Flasher 
  // and initializes the member variables and state
  public:
  Flasher(int pin, long on, long off)
  {
  ledPin = pin;
  pinMode(ledPin, OUTPUT);     
    
  OnTime = on;
  OffTime = off;
  
  ledState = LOW; 
  previousMillis = 0;
  }
 
  void Update(unsigned long currentMillis)
  {
    if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
    {
      ledState = LOW;  // Mark it as off
      previousMillis = currentMillis;  // Remember the time
      digitalWrite(ledPin,ledState?HIGH:LOW);
      //PORTA.OUTCLR = PIN2_bm;  // turn off the output
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime) && ( OnTime > 0) )
    {
      ledState = HIGH;  // mark it as on
      previousMillis = currentMillis;   // Remember the time
      digitalWrite(ledPin,ledState?HIGH:LOW);
      //PORTA.OUTSET = PIN2_bm;   // turn on the output
    }
  }
  void dutyCycle(long on,long off){
    OnTime = on;
    OffTime = off;
   }
};
