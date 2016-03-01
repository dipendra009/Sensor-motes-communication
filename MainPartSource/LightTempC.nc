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
  uses interface Timer<TMilli> as Timer1;
  uses interface Leds;
  uses interface Boot;
  uses interface Read<uint16_t> as Temp;
  uses interface Read<uint16_t> as Light;
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
  uint16_t lux;
  event void Boot.booted()
  {
    call RadioControl.start();
    
  }

  event void RadioControl.startDone(error_t err) {
    if (err == SUCCESS) {
       call Timer0.startPeriodic(LIGHTFREQ);
       call Timer1.startPeriodic(TEMPFREQ);
    }
  }
event void RadioControl.stopDone(error_t err) {
  }

  event void Timer0.fired()
  {
    call Light.read();    
  }
  event void Timer1.fired()
  {
    call Temp.read();    
  }
  
  
  event void Light.readDone(error_t result, uint16_t data)
  {
    lux = 2.5 * 6250.0 * (data/4096.0);

    printf("\nLuminosity is: %d\r",lux);
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
          
  }


  event void Temp.readDone(error_t result, uint16_t data)
  {
    radio_sense_msg_t* rsm;
    uint16_t celsius = -39.6 + (0.01 * data);
    uint16_t farenheit = (((9.0 * celsius)/5)+32);
    RADFREQ %= RADIOFREQ;
    RADFREQ += TEMPFREQ;
    
    printf("\nTemperature is: %d\r", farenheit);
    printf("\nRADFREQ is : %d\r", RADFREQ);
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
          rsm->temp = farenheit;
          rsm->light = lux;
          printf("\nSending l: %d t: %d\r", rsm->temp, rsm->light);
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
}