#include<Timer.h>
#include"TwoTelosb.h"
#include <stdio.h>
#include <string.h>

#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define RADIOFREQ 4000

module TwoTelosbC
{
	uses //General interface
	{
	     interface Boot;
         interface Leds;
         interface Timer<TMilli> as Timer0;
         interface Timer<TMilli> as Timer1;
         
	}
	
	uses//Read for temperature
	{	
	    interface Read<uint16_t> as TempRead;
		
		interface Read<uint16_t> as LightRead;
		
		interface Read<uint16_t> as HumidRead;
	}	
	
	uses //Radio
	{
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;
		//interface Receive;
		
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
		call Timer0.startPeriodic(1000);
		call Timer1.startPeriodic(2000);
		call AMControl.start();
	}

	event void AMSend.sendDone(message_t *msg, error_t error)
	{
		if(msg == &_packet)
		{
			_radioBusy = FALSE;
		}
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
/*
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
*/
	event void Timer0.fired()
	{
		if(call LightRead.read() == SUCCESS) 
		{
			call Leds.led0On();
		}
	}

	event void Timer1.fired()
	{
		if(call TempRead.read() == SUCCESS)
		{
			call Leds.led0On();
		}
	
		
		if(call HumidRead.read() == SUCCESS) 
		{
			call Leds.led0On();
		}
	}

	event void TempRead.readDone(error_t result, uint16_t val)
	{
		RADFREQ_1 += TEMPFREQ;
		if(result == SUCCESS)
		{
			centiGrade = (-39.0 +0.01 *val);
			
			printf("Current temp is: %d \r\n",centiGrade);
			
			if(centiGrade > 29.44) 
			{
			    call Leds.led1On();
			}
			else
			{
				call Leds.led1Off();
			}	
		}	
		else
		{
			printf("Error reading from sensor! \r\n");
		}
		
		if (RADFREQ_1 == RADIOFREQ)
		{
        RADFREQ_1 = 0;	
		if(_radioBusy == FALSE)
		{
			//Creating the packet
		    TwoTelosbMsg_t* msg0 = call Packet.getPayload(& _packet , sizeof(TwoTelosbMsg_t));
		
		    msg0->NodeId = TOS_NODE_ID;
		    msg0->Temp = centiGrade;
		
		    //Sending the packet
		    if(call AMSend.send(AM_BROADCAST_ADDR, & _packet,  sizeof(TwoTelosbMsg_t)) == SUCCESS)
		    {
			    _radioBusy = TRUE;
		    }
		}
		}
	}

	event void LightRead.readDone(error_t result, uint16_t val)
	{
		RADFREQ_2 += LIGHTFREQ;
		if(result == SUCCESS)
		{
			luminance = 2.5 * (val/4096.0)*6250.0;
			
			printf("Current light is: %d \r\n", luminance);
			
			if(luminance < 5) 
			{
			    call Leds.led2On();
			}
			else
			{
				call Leds.led2Off();
			}
	     }   
	    else
	    {
	    	printf("Eroor reading from sensor! \r\n");
	    }
	    
	    if(RADFREQ_2 == RADIOFREQ)
	    {RADFREQ_2 = 0;
	    if(_radioBusy == FALSE)
		{
			//Creating the packet
		    TwoTelosbMsg_t* msg1 = call Packet.getPayload(& _packet , sizeof(TwoTelosbMsg_t));
		
		    msg1->NodeId = TOS_NODE_ID;
		    msg1->Lumi = luminance;
		
		    //Sending the packet
		    if(call AMSend.send(AM_BROADCAST_ADDR, & _packet,  sizeof(TwoTelosbMsg_t)) == SUCCESS)
		    {
			    _radioBusy = TRUE;
		    }
		}
		}
	}

	event void HumidRead.readDone(error_t result, uint16_t val)
	{
		RADFREQ_3 += TEMPFREQ;
		if(result == SUCCESS)
		{
			humidity = (-4 + 0.0405 * val) + (-0.0000028 * pow(val, 2));
			
			printf("Current Humidity is: %d \r\n", humidity);
	    }
	    else
	    {
	    	printf("Error reading from sensor! \r\n");
	    }
	    
	   if(RADFREQ_3==RADIOFREQ)
	   {RADFREQ_3=0;
	   if(_radioBusy == FALSE)
		{
			//Creating the packet
		    TwoTelosbMsg_t* msg1 = call Packet.getPayload(& _packet , sizeof(TwoTelosbMsg_t));
		
		    msg1->NodeId = TOS_NODE_ID;
		    msg1->Humid = humidity;
		
		    //Sending the packet
		    if(call AMSend.send(AM_BROADCAST_ADDR, & _packet,  sizeof(TwoTelosbMsg_t)) == SUCCESS)
		    {
			    _radioBusy = TRUE;
		    }
		}
		}
	}
}
