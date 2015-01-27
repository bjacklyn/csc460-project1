#include "Arduino.h"
#include "scheduler.h"

int test_pin = 5;

const int idle_pin = 7;
const int servo_pin = 9;
const int button_switch_pin = 3;
const int button_movement_pin = 0;
const int transmitter_pin = 13;

const long debounce_delay = 50;                // the debounce time for button presses; increase if the output flickers

volatile long last_debounce_time = 0;          // the last time the output pin was toggled
volatile int servo_pulse_width = 1500;         // microseconds pulse length (500-2400);

volatile int button_switch_state = LOW;        // the current reading from the input pin
volatile int last_button_switch_state = LOW;   // the previous reading from the input pin

volatile int transmitter_send = 0;
volatile int transmitterState = 0;
volatile int transmitterCounter = 8;
volatile int transmitterNumber[8] = {0, 1, 0, 0, 0, 1, 0, 0}; // 65

// task function for button_movement
void poll_button_movement_task()
{
   int val = analogRead(button_movement_pin); // reads the value of the potentiometer (value between 0 and 1023) 
   val = map(val, 0, 1023, 0, 10);            // scale it to use it with the servo (value between 0 and 180) 
   servo_pulse_width = val * 180 + 900;
}

// task function for servo_control
void control_servo_task()
{
   digitalWrite(servo_pin, HIGH);
   delayMicroseconds(servo_pulse_width);      // waits for the servo to get there 
   digitalWrite(servo_pin, LOW);
}


// task function for PulsePin task
void poll_button_switch_task()
{
   // read the state of the switch
   int switch_reading = digitalRead(button_switch_pin);
   
   if (switch_reading != last_button_switch_state) {
     // reset the debouncing timer
     last_debounce_time = millis();
   } 
  
   if ((millis() - last_debounce_time) > debounce_delay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (switch_reading != button_switch_state) {
      button_switch_state = switch_reading;
      if (button_switch_state == LOW) {
        transmitter_send = 1;
      }
    }
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  last_button_switch_state = switch_reading;
}

// task function for PulsePin task
void transmitter_task()
{
  // re-enable timer interrupts
  if (transmitter_send) {
     digitalWrite(test_pin, HIGH);
     digitalWrite(test_pin, LOW);
     cli();
     transmitter_send = 0;
     // enable interupt A for timer 3.
     TIMSK3 |= (1<<OCIE3A);
     // clears outstanding interrupts
     TIFR3 |= (1<<OCF3A);
     // sets timer to 0
     TCNT3 = 0;
     sei();
  }
}

ISR(TIMER3_COMPA_vect)
{
  if (transmitterState == 0) {
     OCR1C = 132;
     TCCR1A |= (1<<COM1C1);
     transmitterState++; 
  } else if (transmitterState == 1) {
     TCCR1A &= ~(1<<COM1C1);
     transmitterState++; 
  } else if (transmitterState == 2) {
    if ( transmitterNumber[transmitterCounter - 1] == 1) {
      TCCR1A |= (1<<COM1C1);
    } else {
      TCCR1A &= ~(1<<COM1C1);
    }
    
    transmitterCounter--;
    if (transmitterCounter == 0) {
      transmitterState++;
    }
  } else if (transmitterState == 3) {
    transmitterCounter = 8;
    transmitterState = 0;

    TCCR1A &= ~(1<<COM1C1);
    TIMSK3 &= ~(1<<OCIE3A);
    TIFR3 |= (1<<OCF3A);
  }
}

// idle task
void idle(uint32_t idle_period)
{
  // this function can perform some low-priority task while the scheduler has nothing to run.
  // It should return before the idle period (measured in ms) has expired.  For example, it
  // could sleep or respond to I/O.
   
  // example idle function that just pulses a pin.
  digitalWrite(idle_pin, HIGH);
  delay(idle_period);
  digitalWrite(idle_pin, LOW);
}

void setup()
{
  pinMode(idle_pin, OUTPUT);
  pinMode(servo_pin, OUTPUT);
  pinMode(test_pin, OUTPUT);
  pinMode(transmitter_pin, OUTPUT);
   
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
  TCCR1A |= (1<<COM1C1); //  TCCR1A &= ~(1<<COM1C1); clear output
  //No prescaler
  TCCR1B |= (1<<CS10);
  
  OCR1A = 421;  //38 Khz
  OCR1C = 0;  //Target
   
  Scheduler_Init();
   
  // Start task arguments are:
  // start offset in ms, period in ms, function callback
 
  Scheduler_StartTask(0, 20, poll_button_movement_task);
  Scheduler_StartTask(5, 20, control_servo_task);
  Scheduler_StartTask(5, 50, poll_button_switch_task);
  Scheduler_StartTask(5, 50, transmitter_task);
  Serial.begin(9600);
}
 
void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {	
    idle(idle_period);
  }
}
