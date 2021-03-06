#include "LightTemp.h"

configuration LightTempAppC
{
}
implementation
{
  components LightTempC, MainC, LedsC, new TimerMilliC(); 
  components new SensirionSht11C() as TempSensor; 
  components new TimerMilliC() as Timer0; 
  components SerialPrintfC;
  components ActiveMessageC;
  components new AMSenderC(AM_RADIO_SENSE_MSG);
  components new AMReceiverC(AM_RADIO_SENSE_MSG);
  
  

  LightTempC.Boot -> MainC;
  LightTempC.Timer0 -> Timer0;
  
  LightTempC.Temp -> TempSensor.Temperature;
  LightTempC.Leds -> LedsC;
  LightTempC.Receive -> AMReceiverC;
  LightTempC.AMSend -> AMSenderC;
  LightTempC.RadioControl -> ActiveMessageC;
  LightTempC.Packet -> AMSenderC;  

}

