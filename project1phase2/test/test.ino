#include "radio.h"

volatile uint8_t rxflag = 0;                   // the received packet flag

//uint8_t station_addr[5] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB }; // packets are transmitted to this address
uint8_t station_addr[5] = {0xAA,0xAA,0xAA,0xAA,0xAA};
 
uint8_t my_addr[5] = {  0x11, 0x3D, 0x51, 0x17, 0xF5 };     // this is this radio's address

radiopacket_t packet;
radiopacket_t recv_packet;

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(10, LOW);
  delay(100);
  digitalWrite(10, HIGH);
  delay(100);

  Radio_Init();
 
  // configure the receive settings for radio pipe 0
  Radio_Configure_Rx(RADIO_PIPE_0, my_addr, ENABLE);
  // configure radio transceiver settings.
  Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);

  Radio_Set_Tx_Addr(station_addr);
  
//  packet.type = MESSAGE;
//  memcpy(packet.payload.message.address, my_addr, RADIO_ADDRESS_LENGTH);
  
//  char message[24] = "ABCDEF";
//  for (int i = 0; i < sizeof(message) - 1; i++) {
//    packet.payload.message.messagecontent[i] = message[i];
//  }
  
//  memcpy(packet.payload.message.messagecontent, message, 10);

  packet.type = COMMAND;
  memcpy(packet.payload.command.sender_address, my_addr, RADIO_ADDRESS_LENGTH);
  packet.payload.command.command = 137;
  packet.payload.command.num_arg_bytes = 4;
  packet.payload.command.arguments[0] = 0;
  packet.payload.command.arguments[1] = 0;
  packet.payload.command.arguments[2] = 0;
  packet.payload.command.arguments[3] = 0;

  Serial.begin(9600);
}

void transmit_packet() {
  Serial.println("sending packet now");
  Radio_Transmit(&packet, RADIO_WAIT_FOR_TX);
}

void loop()
{
  delay(500);
  transmit_packet();
  
  if (rxflag == 1)
  {
    Radio_Receive(&recv_packet);
    rxflag = 0;
  }
}

void radio_rxhandler(uint8_t pipe_number)
{
  Serial.println("recieved packet back");
  rxflag = 1;
}
