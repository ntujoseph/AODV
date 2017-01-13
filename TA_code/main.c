// This is the code of the source device!

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "autonet.h"

// You can define the period of retransmittion time of the source device
#define RETRANSMIT_PERIOD 200

int main(void)
{
	uint8_t src_packet[128] = {0x05, 0x30, 0x00, 0x00, 0x0A};
	
	uint8_t rcvd_msg[128] = {0};
	uint8_t rcvd_payload[128] = {0};
	uint8_t rcvd_length;
	uint8_t rcvd_payloadLength;
	uint8_t rcvd_rssi;
	
	uint8_t Type;
	uint16_t Addr;
	uint8_t radio_channel;
	uint16_t radio_panID;
	
	Type = Type_Light;
	Addr = 0x0001;
	radio_channel = 18;
	radio_panID = 0x00AA;
	
	Initial(Addr, Type, radio_channel, radio_panID);
	setTimer(1,RETRANSMIT_PERIOD,UNIT_MS);
	
	while(1){
		
		// Periodically send the msg
		if(checkTimer(1)){
			RF_Tx(0xFFFF,src_packet,5);
		}
		
		// When received some packet
		if(RF_Rx(rcvd_msg, &rcvd_length, &rcvd_rssi)){
			getPayloadLength(&rcvd_payloadLength, rcvd_msg);
			getPayload(rcvd_payload, rcvd_msg, rcvd_payloadLength);
			
			// Check 1)header, 2)sequence number, 3)isACK field
			if(rcvd_payload[0]==0x05 && rcvd_payload[1]==0x30 && 
				 rcvd_payload[2]==src_packet[2] && rcvd_payload[3]==1){
				src_packet[2]++;
				// Change the payload here
			}
		}
		
		if(src_packet[2]==0x14)
			break;
	}
	
	while(1){
		if(checkTimer(1)){
			setGPIO(1,1);
		}	
	}
}
