const int buttonPin = 2;    // the number of the pushbutton pin
const int transmitterPin = 13;

volatile int transmitterState = 0;
volatile int transmitterCounter = 8;
volatile int transmitterNumber[8] = {0, 132, 0, 0, 0, 132, 0, 0}; // 65

// Variables will change:
int buttonState = LOW;      // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(transmitterPin, OUTPUT);

  //Clear timer config.
  TCCR3A = 0;
  TCCR3B = 0;  
  //Set to CTC (mode 4)
  TCCR3B |= (1<<WGM32);
  
  //Set prescaler to 256
  TCCR3B |= (1<<CS30);
  
  //Set TOP value (0.05 seconds)
  OCR3A = 8000;
  
  //Enable interupt A for timer 3.
  TIMSK3 |= (1<<OCIE3A);
  
  //Set timer to 0 (optional here).
  TCNT3 = 0;
  
  //=======================
  
  //PWM  
  //Clear timer config
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 &= ~(1<<OCIE1C);
  //Set to Fast PWM (mode 15)
  TCCR1A |= (1<<WGM10) | (1<<WGM11);
  TCCR1B |= (1<<WGM12) | (1<<WGM13);
  	
  //Enable output C.
  TCCR1A |= (1<<COM1C1);
  //No prescaler
  TCCR1B |= (1<<CS10);
  
  OCR1A = 421;  //38 Khz
  OCR1C = 0;  //Target
  Serial.begin(9600);
}

volatile int flag = 0;

ISR(TIMER3_COMPA_vect)
{
  if (flag == 1) {
    if (transmitterState == 0) {
       OCR1C = 132;
       transmitterState++; 
    } else if (transmitterState == 1) {
       OCR1C = 0;
       transmitterState++; 
    } else if (transmitterState == 2) {
      OCR1C = transmitterNumber[transmitterCounter - 1];
      transmitterCounter--;
      if (transmitterCounter == 0) {
        transmitterState++;
      }
    } else if (transmitterState == 3) {
        transmitterCounter = 8;
        transmitterState = 0;
        flag = 0;
        OCR1C = 0;
    }
  }
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        flag = 1;
      }
      Serial.println(buttonState);
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
}

