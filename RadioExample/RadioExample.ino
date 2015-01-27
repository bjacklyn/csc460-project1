#include "Arduino.h"
#include "radio.h"
 
volatile uint8_t rxflag = 0;
 
// packets are transmitted to this address
uint8_t station_addr[5] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB };
 
// this is this radio's address
uint8_t my_addr[5] = { 0x55, 0x55, 0x55, 0x55, 0x55 };
 
radiopacket_t packet;
 
int main()
{
	init();
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
 
	// put some data into the packet
	packet.type = MESSAGE;
	memcpy(packet.payload.message.address, my_addr, RADIO_ADDRESS_LENGTH);
	//packet.payload.message.messageid = 88;
	snprintf((char*)packet.payload.message.messagecontent, sizeof(packet.payload.message.messagecontent), "CDEFGHIJKLMNOPQRSTUVWXYZ");
 
	// The address to which the next transmission is to be sent
	Radio_Set_Tx_Addr(station_addr);
 
	// send the data
	Radio_Transmit(&packet, RADIO_WAIT_FOR_TX);
 
	// wait for the ACK reply to be transmitted back.
	for (;;)
	{
		if (rxflag)
		{
			// remember always to read the packet out of the radio, even
			// if you don't use the data.
			if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS)
			{
				// if there are no more packets on the radio, clear the receive flag;
				// otherwise, we want to handle the next packet on the next loop iteration.
				rxflag = 0;
			}
			if (packet.type == ACK)
			{
				digitalWrite(13, HIGH);
			}
		}
	}
	for (;;);
	return 0;
}
 
void radio_rxhandler(uint8_t pipe_number)
{
	rxflag = 1;
}
