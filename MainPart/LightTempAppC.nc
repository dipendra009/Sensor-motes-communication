#include "LightTemp.h"

configuration LightTempAppC
{
}
implementation
{
  components LightTempC, MainC, LedsC, new TimerMilliC();
  components new HamamatsuS10871TsrC() as Light; 
  components new SensirionSht11C() as TempSensor; 
  components new TimerMilliC() as Timer0; 
  components new TimerMilliC() as Timer1; 
  components SerialPrintfC;
  components ActiveMessageC;
  components new AMSenderC(AM_RADIO_SENSE_MSG);
  components new AMReceiverC(AM_RADIO_SENSE_MSG);
  
  LightTempC.Boot -> MainC;
  LightTempC.Timer0 -> Timer0;
  LightTempC.Timer1 -> Timer1;
  
  LightTempC.Temp -> TempSensor.Temperature;
  LightTempC.Light -> Light;
  LightTempC.Leds -> LedsC;
  LightTempC.Receive -> AMReceiverC;
  LightTempC.AMSend -> AMSenderC;
  LightTempC.RadioControl -> ActiveMessageC;
  LightTempC.Packet -> AMSenderC;  

}

