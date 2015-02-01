#include "Arduino.h"
#include "scheduler.h"
#include "radio.h"
 
#define HIGH_BYTE(x) x>>8
#define LOW_BYTE(x) x&0xFF

const int idle_pin = 7;
const int radio_pin = 10;
const int button_switch_pin = 3;
const int button_movement_pin = 0;
const int button_rotation_pin = 1;

const long debounce_delay = 50;                // the debounce time for button presses; increase if the output flickers

volatile long last_debounce_time = 0;          // the last time the output pin was toggled

volatile uint8_t rxflag = 0;                   // the received packet flag

//uint8_t station_addr[5] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB }; // packets are transmitted to this address
uint8_t station_addr[5] = {0xAA,0xAA,0xAA,0xAA,0xAA};
 
uint8_t my_addr[5] = {  0x11, 0x3D, 0x51, 0x17, 0xF5 };     // this is this radio's address

radiopacket_t send_packet;
radiopacket_t recv_packet;

volatile int button_switch_state = LOW;        // the current reading from the input pin
volatile int last_button_switch_state = LOW;   // the previous reading from the input pin

volatile int ir_fire_flag = 0;
volatile int roomba_speed = 0;
volatile int roomba_rotation = 0x8000;

// task function for button_movement
void poll_button_movement_task()
{
  int movement = analogRead(button_movement_pin); 
  int rotation = analogRead(button_rotation_pin); 

  if (movement > 400 && movement < 600) {
    roomba_speed = 0;
  } 
  else if (movement < 400) {
    roomba_speed = -200;
  } 
  else {
    roomba_speed = 200;   
  }
  
  if (rotation > 400 && rotation < 600) {
    roomba_rotation = 0x8000;
  } 
  else if (rotation < 400) {
    roomba_speed = 200;
    roomba_rotation = 0xFFFF;
  } 
  else {
    roomba_speed = 200;
    roomba_rotation = 0;
  }
  
  send_movement_packet();
}

void send_movement_packet() 
{
  send_packet.type = COMMAND;
  memcpy(send_packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
  send_packet.payload.command.command = 137;
  send_packet.payload.command.num_arg_bytes = 4;
  send_packet.payload.command.arguments[0] = HIGH_BYTE(roomba_speed);
  send_packet.payload.command.arguments[1] = LOW_BYTE(roomba_speed);
  send_packet.payload.command.arguments[2] = HIGH_BYTE(roomba_rotation);
  send_packet.payload.command.arguments[3] = LOW_BYTE(roomba_rotation);

  Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);
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
        ir_fire_flag = 1;
      }
    }
  }
  
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  last_button_switch_state = switch_reading;
}

void ir_task()
{
  if (ir_fire_flag) {
    
     send_packet.type = IR_COMMAND;
     memcpy(send_packet.payload.ir_command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
     send_packet.payload.ir_command.ir_command = SEND_BYTE;
     send_packet.payload.ir_command.ir_data = 66;
     send_packet.payload.ir_command.servo_angle = 0;

     Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);
     
     ir_fire_flag = 0;
  }
}

void receive_packets_task()
{
  if (rxflag)
  {
    if (Radio_Receive(&recv_packet) != RADIO_RX_MORE_PACKETS)
    {
      // if there are no more packets on the radio, clear the receive flag;
      // otherwise, we want to handle the next packet on the next loop iteration.
      rxflag = 0;
    }
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
  pinMode(radio_pin, OUTPUT);

  digitalWrite(radio_pin, LOW);
  delay(100);
  digitalWrite(radio_pin, HIGH);
  delay(100);

  Radio_Init();
 
  // configure the receive settings for radio pipe 0
  Radio_Configure_Rx(RADIO_PIPE_0, my_addr, ENABLE);
  // configure radio transceiver settings.
  Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);

  Radio_Set_Tx_Addr(station_addr);

  Scheduler_Init();
   
  // Start task arguments are:
  // start offset in ms, period in ms, function callback
 
  Scheduler_StartTask(0, 20, poll_button_movement_task);
  Scheduler_StartTask(0, 50, poll_button_switch_task);
  Scheduler_StartTask(5, 50, ir_task);
  Scheduler_StartTask(10, 200, receive_packets_task);
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

void radio_rxhandler(uint8_t pipe_number)
{
  rxflag = 1;
}
