#include<stdio.h>
#include "LightTemp.h"


#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define LIGHTLIMIT 30
#define TEMPLIMIT 85
#define RADIOFREQ 4000

module LightTempC @safe()
{
  uses interface Leds;
  uses interface Boot;
  uses interface Receive;
  uses interface AMSend;
  uses interface Packet;
  uses interface SplitControl as RadioControl;
    
}

implementation
{
  event void Boot.booted()
  {
    call RadioControl.start();
    
  }

  event void RadioControl.startDone(error_t err) {
    if (err == SUCCESS) {
  }
  }
event void RadioControl.stopDone(error_t err) {
  }
  
  
  event message_t* Receive.receive(message_t* bufPtr, 
           void* payload, uint8_t len) 
  {
    uint16_t luxv, tempv;
    if (len != sizeof(radio_sense_msg_t)) {
      return bufPtr;
    }
    else 
    {
      radio_sense_msg_t* rsm = (radio_sense_msg_t*)payload;
      luxv = rsm->light;
      tempv = rsm->temp;
      printf("\nData are : %d %d", luxv, tempv);
      
      if (luxv < LIGHTLIMIT)
      {
        call Leds.led2On();
      }
      else{
        call Leds.led2Off();
      }
      if (tempv > TEMPLIMIT)
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



