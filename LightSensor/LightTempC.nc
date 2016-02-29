#include<stdio.h>
#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define LIGHTLIMIT 30
#define TEMPLIMIT 85
#define RADIOFREQ 4000

module LightTempC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  //uses interface Timer<TMilli> as Timer2;
  uses interface Leds;
  uses interface Boot;
  uses interface Read<uint16_t> as Light;
  //uses interface Read<uint16_t> as Temp;
  interface Receive;
  interface AMSend;
  interface Packet;
  interface SplitControl as RadioControl;
    
}

implementation
{
  uint16_t RADFREQ = 0;
  message_t packet;
  bool lock = FALSE;
  event void Boot.booted()
  {
    call RadioControl.start();
    //call Timer0.startPeriodic(LIGHTFREQ);
    //call Timer1.startPeriodic(TEMPFREQ);
  }

  event void RadioControl.startDone(error_t err) {
    if (err == SUCCESS) {
       call Timer0.startPeriodic(LIGHTFREQ);
    }
  }
event void RadioControl.stopDone(error_t err) {
  }

  event void Timer0.fired()
  {
    call Light.read();    
  }
  
  
  event void Light.readDone(error_t result, uint16_t data)
  {
    uint16_t lux = 2.5 * 6250.0 * (data/4096.0);

    printf("\nLuminosity is: %d",lux);
    RADFREQ += LIGHTFREQ;
    if (result == SUCCESS)
    {
      if (lux < LIGHTLIMIT)
      {
         call Leds.led2On();
      }
      else
      {
      	call Leds.led2Off();
      }
    }
    if (RADFREQ == RADIOFREQ)
    {
      RADFREQ =0;
      if (lock) return;
      else
      {

        rsm = (radio_sense_msg_t*)call Packet.getPayload(&packet, sizeof(radio_sense_msg_t));
        if (rsm == NULL) {
          return;
        }
        rsm->error = result;
        rsm->data = lux;
        if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_sense_msg_t)) == SUCCESS) 
        {
          lock = TRUE;
        }
      }
    }      
  }

  /*
  event void Temp.readDone(error_t result, uint16_t data)
  {
    uint16_t celsius = -39.6 + (0.01 * data);
    uint16_t farenheit = (((9.0 * celsius)/5)+32);
    printf("\nTemperature is: %d", farenheit);
    if (result == SUCCESS)
    {
      if (farenheit > TEMPLIMIT)
      {
	call Leds.led1On();
      }
      else
      {
        call Leds.led1Off();
      }

    } 
  } */

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      lock = FALSE;
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, 
           void* payload, uint8_t len) 
  {
    uint16_t farenheit;
    if (len != sizeof(radio_sense_msg_t)) {
      return bufPtr;
    }
    else 
    {
      radio_sense_msg_t* rsm = (radio_sense_msg_t*)payload;
      farenheit = rsm->data;
      printf("\nTemperature is: %d", farenheit);
      
      if (farenheit >= TEMPLIMIT)
      {
        call Leds.led1On();
      }
      else{
        call Leds.led1Off();
      }
    }
    return bufPtr;
    
  }  
} 



