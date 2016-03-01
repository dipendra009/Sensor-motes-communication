#include<Timer.h>
#include"TwoTelosb.h"
#include <stdio.h>
#include <string.h>

#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define RADIOFREQ 4000

module TwoTelosbC
{
	uses 
	{
	     interface Boot;
         interface Leds;
         
	}
	
	uses
	{
		interface SplitControl as AMControl;
		interface Receive;
		
	}
}

implementation
{
	uint16_t RADFREQ_1 = 0;
	uint16_t RADFREQ_2 = 0;
	uint16_t RADFREQ_3 = 0;
	uint16_t centiGrade;
    uint16_t luminance;
    uint16_t humidity;
	bool _radioBusy = FALSE;
	message_t _packet;

	event void Boot.booted()
	{
		call AMControl.start();
	}

	event void AMControl.stopDone(error_t error)
	{
		// TODO Auto-generated method stub
	}

	event void AMControl.startDone(error_t error)
	{
		if(error == SUCCESS)
		{
			call Leds.led0On();
		}	
		else
		{
			call AMControl.start();
		}	
	}

	event message_t * Receive.receive(message_t *msg, void *payload, uint8_t len)
	{
		if(len == sizeof(TwoTelosbMsg_t))
		{
			TwoTelosbMsg_t * incomingPacket = (TwoTelosbMsg_t*) payload;
			
			uint16_t Temp = incomingPacket->Temp;
			uint16_t Lumi = incomingPacket->Lumi;
			uint16_t Humid = incomingPacket->Humid;
			
			//mimicking
			if(Temp > 29.44) 
			{
			    call Leds.led1On();
			}
			else
			{
				call Leds.led1Off();
			}
			
			if(Lumi < 5) 
			{
			    call Leds.led2On();
			}
			else
			{
				call Leds.led2Off();
			}	
		}	
		return msg;
	}
}
