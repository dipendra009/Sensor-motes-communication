#include<stdio.h>
#define LIGHTFREQ 1000
#define TEMPFREQ 2000
#define RADIOFREQ 4000
#define LIGHTLIMIT 30
#define TEMPLIMIT 85

module LightTempC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Timer<TMilli> as Timer1;
  uses interface Timer<TMilli> as Timer2;
  uses interface Leds;
  uses interface Boot;
  interface AMSend;
  interface SplitControl as AMControl;
  interface Packet;
  uses interface Read<uint16_t> as Light;
  uses interface Read<uint16_t> as Temp;
}

implementation
{
  message_t packet;

  bool locket;
  uint16_t counter = 0;

  event void Boot.booted()
  {
    call AMControl.start();
    call Timer0.startPeriodic(LIGHTFREQ);
    call Timer1.startPeriodic(TEMPFREQ);
  }

  event void AMControl.startDone(error_t err) 
  {
    if (err == SUCCESS) {
      call Timer2.startPeriodic(RADIOFREQ);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) 
  {
    // do nothing
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
    uint16_t lux = 2.5 * 6250.0 * (data/4096.0);
    printf("\nLuminosity is: %d",lux);

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
  }  

}

