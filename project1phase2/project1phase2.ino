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

enum packet_send {
  SEND,
  STOP
};

typedef enum packet_send PACKET_SEND;

volatile PACKET_SEND stop_packet_send;

enum speed_state {
  SLOW,
  MEDIUM,
  FAST,
  REVERSE_SLOW,
  REVERSE_MEDIUM,
  REVERSE_FAST,
  NO_SPEED
};

typedef enum speed_state SPEED_STATE;

volatile SPEED_STATE last_speed;

volatile SPEED_STATE current_speed;

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
volatile int song_fire_flag = 0;
volatile int song_rotation = 0;
volatile int no_speed_flag = 0;
volatile int no_rotation_flag = 0;
volatile int stop_packet_insurance = 0;
volatile int roomba_speed = 0;
volatile int roomba_rotation = 0x8000;
volatile int acceleration = 10;

// controls movement parameters
void movement_set(uint16_t movement) {
  if (movement > 450 && movement < 550) {
 //   roomba_speed = 0;
    no_speed_flag = 0;
    current_speed = NO_SPEED;
    last_speed = NO_SPEED;
    acceleration_calc();    
  } 
  else if (movement > 350 && movement < 450) {
    current_speed = REVERSE_SLOW;
    //roomba_speed = -150;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  } 
  else if(movement > 550 && movement < 650){
    current_speed = SLOW;
    //roomba_speed = 150;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement > 150 && movement < 350) {
    current_speed = REVERSE_MEDIUM;
    //roomba_speed = -250;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement > 650 && movement < 950){
    current_speed = MEDIUM;
    //roomba_speed = 250;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement < 150) {
    current_speed = REVERSE_FAST;
    //roomba_speed = -350;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  } else {
    current_speed = FAST;
    //roomba_speed = 350;
    acceleration_calc();
    no_speed_flag = 1;
    stop_packet_insurance = 0;
  }
}

void acceleration_calc() {
  switch (last_speed) {
      case REVERSE_SLOW:
        acceleration_reverse_slow();
        break;
      case REVERSE_MEDIUM:
        acceleration_reverse_medium();
        break;
      case REVERSE_FAST:
        acceleration_reverse_fast();
        break;
      case SLOW:
        acceleration_slow();
        break;
      case MEDIUM:
        acceleration_medium();
        break;
      case FAST:
        acceleration_fast();
        break;
      default:
        acceleration_no_speed();
  }
}

void acceleration_no_speed() {
  if(current_speed != last_speed && (current_speed == FAST || current_speed == MEDIUM || current_speed == SLOW)) {
    roomba_speed += acceleration;
    if(roomba_speed > 500) {
      last_speed = FAST;
    } else if(roomba_speed > 350) {
      last_speed = MEDIUM;
    } else if(roomba_speed > 150) {
      last_speed = SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed != last_speed && (current_speed == REVERSE_FAST || current_speed == REVERSE_MEDIUM || current_speed == REVERSE_SLOW)) {
    roomba_speed -= acceleration;
    if(roomba_speed < -500) {
      last_speed = REVERSE_FAST;
    } else if(roomba_speed < -350) {
      last_speed = REVERSE_MEDIUM;
    } else if(roomba_speed < -150) {
      last_speed = REVERSE_SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  }  else {
    last_speed = NO_SPEED;
    roomba_speed = 0;
  }
}

void acceleration_reverse_slow() {
  if(current_speed != last_speed && (current_speed == REVERSE_FAST || current_speed == REVERSE_MEDIUM || current_speed == REVERSE_SLOW)) {
    roomba_speed -= acceleration;
    if(roomba_speed < -500) {
      last_speed = REVERSE_FAST;
    } else if(roomba_speed < -350) {
      last_speed = REVERSE_MEDIUM;
    } else if(roomba_speed < -150) {
      last_speed = REVERSE_SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed == last_speed) {
    if(roomba_speed < -180) {
      roomba_speed += acceleration;
    } else if(roomba_speed > -140) {
      roomba_speed -= acceleration;
    }
   last_speed = REVERSE_SLOW; 
  }
}

void acceleration_slow() {
  if(current_speed != last_speed && (current_speed == FAST || current_speed == MEDIUM || current_speed == SLOW)) {
    roomba_speed += acceleration;
    if(roomba_speed > 500) {
      last_speed = FAST;
    } else if(roomba_speed > 350) {
      last_speed = MEDIUM;
    } else if(roomba_speed > 150) {
      last_speed = SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed == last_speed) {
    if(roomba_speed > 180) {
      roomba_speed -= acceleration;
    } else if(roomba_speed < 140) {
      roomba_speed += acceleration;
    }
    last_speed = SLOW;
  }
}

void acceleration_reverse_medium() {
  if(current_speed != last_speed && (current_speed == REVERSE_FAST || current_speed == REVERSE_MEDIUM)) {
    roomba_speed -= acceleration;
    if(roomba_speed < -500) {
      last_speed = REVERSE_FAST;
    } else if(roomba_speed < -350) {
      last_speed = REVERSE_MEDIUM;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed != last_speed && current_speed == REVERSE_SLOW) {
    roomba_speed += acceleration;
    if(roomba_speed < -150) {
      last_speed = REVERSE_SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed == last_speed) {
    if(roomba_speed < -380) {
      roomba_speed += acceleration;
    } else if(roomba_speed > -340) {
      roomba_speed -= acceleration;
    }
    last_speed = NO_SPEED;
  }
}

void acceleration_medium() {
  if(current_speed != last_speed && (current_speed == FAST || current_speed == MEDIUM)) {
    roomba_speed += acceleration;
    if(roomba_speed > 500) {
      last_speed = FAST;
    } else if(roomba_speed > 350) {
      last_speed = MEDIUM;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed != last_speed && current_speed == SLOW) {
    roomba_speed -= acceleration;
    if(roomba_speed > 150){
      last_speed = SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed == last_speed) {
    if(roomba_speed > 380) {
      roomba_speed += acceleration;
    } else if(roomba_speed < 340) {
      roomba_speed -= acceleration;
    }
    last_speed = NO_SPEED;
  }
}

void acceleration_reverse_fast() {
  if(current_speed != last_speed && current_speed == REVERSE_FAST){
    roomba_speed -= acceleration;
    if(roomba_speed < -500){
      last_speed = REVERSE_FAST;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed != last_speed && (current_speed == REVERSE_MEDIUM || current_speed == REVERSE_SLOW)){
    roomba_speed += acceleration;
    if(roomba_speed < -350) {
      last_speed = REVERSE_MEDIUM;
    } else if(roomba_speed < -500) {
      last_speed = REVERSE_FAST;
    } else {
      last_speed = NO_SPEED;
    }
  }
}

void acceleration_fast() {
  if(current_speed != last_speed && current_speed == FAST) {
    roomba_speed += acceleration;
    if(roomba_speed > 500) {
      last_speed = FAST;
    } else {
      last_speed = NO_SPEED;
    }
  } else if(current_speed != last_speed && (current_speed == SLOW || current_speed == MEDIUM)) {
    roomba_speed -= acceleration;
    if(roomba_speed > 350) {
      last_speed = MEDIUM;
    } else if(roomba_speed > 150){
      last_speed = SLOW;
    } else {
      last_speed = NO_SPEED;
    }
  }
}

// controls rotation parameters
void rotation_set(uint16_t rotation) {
  if (rotation > 450 && rotation < 550) {
    roomba_rotation = 0x8000;
    no_rotation_flag = 0;
  } 
  else if (rotation > 350 && rotation < 450) {
    roomba_speed = 150;
    roomba_rotation = 0xFFFF;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  } 
  else if (rotation > 150 && rotation < 350) {
    roomba_speed = 250;
    roomba_rotation = 0xFFFF;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  }
  else if (rotation < 150) {
    roomba_speed = 350;
    roomba_rotation = 0xFFFF;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  }
  else if (rotation > 550 && rotation < 650) {
    roomba_speed = 150;
    roomba_rotation = 0;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  }
  else if (rotation > 650 && rotation < 950) {
    roomba_speed = 250;
    roomba_rotation = 0;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  } 
  else {
    roomba_speed = 350;
    roomba_rotation = 0;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  }
}

// controls angle parameters
void angle_set(uint16_t movement, uint16_t rotation) {
  if(movement > 950 && rotation > 950) {
    roomba_speed = 300;
    roomba_rotation = 400;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement > 950 && rotation < 150) {
    roomba_speed = 300;
    roomba_rotation = -400;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement < 150 && rotation < 150) {
    roomba_speed = -300;
    roomba_rotation = 400;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  } else if(movement < 150 && rotation > 950) {
    roomba_speed = -300;
    roomba_rotation = -400;
    no_rotation_flag = 1;
    stop_packet_insurance = 0;
  }
}

// task function for button_movement
void poll_button_movement_task()
{
  int movement = analogRead(button_movement_pin); 
  int rotation = analogRead(button_rotation_pin); 

  movement_set(movement);
  rotation_set(rotation);
  angle_set(movement, rotation);
  // checks if it should send the packet sends the packet
  if(no_speed_flag == 1 || no_rotation_flag == 1){
    send_movement_packet();
    stop_packet_send = SEND;
  } else {
    if(stop_packet_send == SEND || stop_packet_insurance < 10) {
      stop_packet_insurance++;
      send_movement_packet();
    }
  }
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

void send_song_packet()
{
  if(song_fire_flag){
    if(song_rotation == 0){
    send_packet.type = COMMAND;
    memcpy(send_packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
    send_packet.payload.command.command = 140;
    send_packet.payload.command.num_arg_bytes = 18;
    send_packet.payload.command.arguments[0] = 1;
    send_packet.payload.command.arguments[1] = 8;

    send_packet.payload.command.arguments[2] = 103; // G
    send_packet.payload.command.arguments[3] = 20;
    send_packet.payload.command.arguments[4] = 102; // F#
    send_packet.payload.command.arguments[5] = 20;
    send_packet.payload.command.arguments[6] = 99; // D#
    send_packet.payload.command.arguments[7] = 20;
    send_packet.payload.command.arguments[8] = 93; // A
    send_packet.payload.command.arguments[9] = 20;
    send_packet.payload.command.arguments[10] = 104; // G#
    send_packet.payload.command.arguments[11] = 20;
    send_packet.payload.command.arguments[12] = 100; // E
    send_packet.payload.command.arguments[13] = 20;
    send_packet.payload.command.arguments[14] = 104; // G#
    send_packet.payload.command.arguments[15] = 20;
    send_packet.payload.command.arguments[16] = 96; // C
    send_packet.payload.command.arguments[17] = 20;

    Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);

    send_packet.type = COMMAND;
    memcpy(send_packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
    send_packet.payload.command.command = 141;
    send_packet.payload.command.num_arg_bytes = 1;
    send_packet.payload.command.arguments[0] = 1;
    
    Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);

    song_fire_flag = 0;
    song_rotation++;
  } else {
    send_packet.type = COMMAND;
    memcpy(send_packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
    send_packet.payload.command.command = 140;
    send_packet.payload.command.num_arg_bytes = 18;
    send_packet.payload.command.arguments[0] = 2;
    send_packet.payload.command.arguments[1] = 8;

    send_packet.payload.command.arguments[2] = 78; // F#
    send_packet.payload.command.arguments[3] = 32;
    send_packet.payload.command.arguments[4] = 74; // D
    send_packet.payload.command.arguments[5] = 32;
    send_packet.payload.command.arguments[6] = 78; // F#
    send_packet.payload.command.arguments[7] = 32;
    send_packet.payload.command.arguments[8] = 74; // D
    send_packet.payload.command.arguments[9] = 32;
    send_packet.payload.command.arguments[10] = 70; // A#
    send_packet.payload.command.arguments[11] = 32;
    send_packet.payload.command.arguments[12] = 78; // F#
    send_packet.payload.command.arguments[13] = 32;
    send_packet.payload.command.arguments[14] = 70; // A#
    send_packet.payload.command.arguments[15] = 32;
    send_packet.payload.command.arguments[16] = 78; // F#
    send_packet.payload.command.arguments[17] = 32;

    Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);

    send_packet.type = COMMAND;
    memcpy(send_packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
    send_packet.payload.command.command = 141;
    send_packet.payload.command.num_arg_bytes = 1;
    send_packet.payload.command.arguments[0] = 2;
    
    Radio_Transmit(&send_packet, RADIO_WAIT_FOR_TX);

    song_fire_flag = 0;
    song_rotation = 0;
  } 
  }
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
        song_fire_flag = 1;
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
     send_packet.payload.ir_command.ir_data = 65;
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
 
  Scheduler_StartTask(0, 100, poll_button_movement_task);
  Scheduler_StartTask(0, 50, poll_button_switch_task);
  Scheduler_StartTask(5, 50, ir_task);
  Scheduler_StartTask(10, 50, send_song_packet);
  Scheduler_StartTask(10, 200, receive_packets_task);
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
  if(no_speed_flag == 0 && no_rotation_flag == 0) {
    stop_packet_send = STOP;
  }
  rxflag = 1;
}
