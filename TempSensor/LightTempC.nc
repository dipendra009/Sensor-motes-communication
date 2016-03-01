#include<stdio.h>
#include "LightTemp.h"


#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define LIGHTLIMIT 30
#define TEMPLIMIT 85
#define RADIOFREQ 4000

module LightTempC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Leds;
  uses interface Boot;
  uses interface Read<uint16_t> as Temp;
  uses interface Receive;
  uses interface AMSend;
  uses interface Packet;
  uses interface SplitControl as RadioControl;
    
}

implementation
{
  uint16_t RADFREQ = 0;
  message_t packet;
  bool lock = FALSE;
  event void Boot.booted()
  {
    call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err) {
    if (err == SUCCESS) {
       call Timer0.startPeriodic(TEMPFREQ);
    }
  }
  event void RadioControl.stopDone(error_t err) {
  }

  event void Timer0.fired()
  {
    call Temp.read();    
  }
  
  
  event void Temp.readDone(error_t result, uint16_t data)
  {
    radio_sense_msg_t* rsm;
    uint16_t celsius = -39.6 + (0.01 * data);
    uint16_t farenheit = (((9.0 * celsius)/5)+32);
    RADFREQ %= RADIOFREQ;
    RADFREQ += TEMPFREQ;
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
      if (RADFREQ == RADIOFREQ)
      {
        RADFREQ = 0;
        if (lock) return;
        else
        {

          rsm = (radio_sense_msg_t*)call Packet.getPayload(&packet, sizeof(radio_sense_msg_t));
          if (rsm == NULL) {
            return;
          }
          rsm->error = result;
          rsm->data = farenheit;
          if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_sense_msg_t)) == SUCCESS) 
          {
            lock = TRUE;
          }
        }
      }
    }
  } 

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      lock = FALSE;
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, 
           void* payload, uint8_t len) 
  {
    uint16_t lux;
    if (len != sizeof(radio_sense_msg_t)) {
      return bufPtr;
    }
    else 
    {
      radio_sense_msg_t* rsm = (radio_sense_msg_t*)payload;
      lux = rsm->data;
      printf("\nLuminosity is: %d", lux);
      
      if (lux <= LIGHTLIMIT)
      {
        call Leds.led2On();
      }
      else{
        call Leds.led2Off();
      }
    }
    return bufPtr;
    
  }  
} 



