#include "Arduino.h"
#include "radio.h"
 
#define HIGH_BYTE(x) x>>8
#define LOW_BYTE(x) x&0xFF

volatile uint8_t rxflag = 0;
 
// packets are transmitted to this address
uint8_t station_addr[5] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB };
//uint8_t station_addr[5] = { 0xAA,0xAA,0xAA,0xAA,0xAA };
 
// this is this radio's address
uint8_t my_addr[5] = {  0x98, 0x76, 0x54, 0x32, 0x10 };
 
radiopacket_t packet;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}

int main()
{     
        
	init();
        setup();
	pinMode(13, OUTPUT);
	pinMode(47, OUTPUT);
 
	digitalWrite(47, LOW);
	delay(100);
	digitalWrite(47, HIGH);
	delay(100);
 
	Radio_Init();
 
	// configure the receive settings for radio pipe 0
	Radio_Configure_Rx(RADIO_PIPE_0, my_addr, ENABLE);
	// configure radio transceiver settings.
	Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);
 
	// put some data into the packet
//	packet.type = COMMAND;
        packet.type = MESSAGE;
 	memcpy(packet.payload.message.address , my_addr, RADIO_ADDRESS_LENGTH);
//        packet.payload.command.command = 137;
//        packet.payload.command.num_arg_bytes = 4;
//        packet.payload.command.arguments[0] = HIGH_BYTE(100);
//        packet.payload.command.arguments[1] = LOW_BYTE(100);
//        packet.payload.command.arguments[2] = HIGH_BYTE(0x8000);
//        packet.payload.command.arguments[3] = LOW_BYTE(0x8000);
//      
	snprintf((char*)packet.payload.message.messagecontent, 25, "Hello world33757S33!");
//	snprintf((char*)packet.payload.message.messagecontent, sizeof(packet.payload.message.messagecontent)+1, "Hello world33363!");

	// The address to which the next transmission is to be sent
	Radio_Set_Tx_Addr(station_addr);

        while (true) {
                Serial.println((char*)packet.payload.message.messagecontent);
          
                 // send the data
	        Radio_Transmit(&packet, RADIO_WAIT_FOR_TX);
                snprintf((char*)packet.payload.message.messagecontent, sizeof(packet.payload.message.messagecontent)+1, "Goodbye world!");
                Serial.println((char*)packet.payload.message.messagecontent);
                Serial.println("got here");
        	// wait for the ACK reply to be transmitted back.
        	for (;;)
        	{
                        Serial.println("stuck");
        		if (rxflag)
        		{
        			// remember always to read the packet out of the radio, even
        			// if you don't use the data.
        			if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS)
        			{
        				// if there are no more packets on the radio, clear the receive flag;
        				// otherwise, we want to handle the next packet on the next loop iteration.
        				rxflag = 0;
                                        break;
        			}
        			if (packet.type == MESSAGE)
        			{
        				digitalWrite(13, HIGH);
        			}
        		}
        	}

//                delay(200);
        }
	for (;;);
	return 0;
}
 
void radio_rxhandler(uint8_t pipe_number)
{
        Serial.println("packet ready");
	rxflag = 1;
}
