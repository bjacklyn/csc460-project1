volatile int flag = 0;
volatile int counter = 7;
volatile int array[8] = {132, 0, 0, 0, 132, 132, 132, 132};


void setup() {
  pinMode(13, OUTPUT);
  
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
  
}

ISR(TIMER3_COMPA_vect)
{
  if (flag == 0) {
     OCR1C = 132;
     flag++; 
  } else if (flag == 1) {
     OCR1C = 0;
     flag++; 
  } else if (flag == 2) {
    OCR1C = array[counter - 1];
    counter--;
    if (counter == 0) {
      counter = 8;
      flag = 0;
    }
  }
}


void loop() {
  
}
