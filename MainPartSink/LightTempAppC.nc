#include "LightTemp.h"

configuration LightTempAppC
{
}
implementation
{
  components LightTempC, MainC, LedsC;
  components SerialPrintfC;
  components ActiveMessageC;
  components new AMSenderC(AM_RADIO_SENSE_MSG);
  components new AMReceiverC(AM_RADIO_SENSE_MSG);
  
  LightTempC.Boot -> MainC;
  
  LightTempC.Leds -> LedsC;
  LightTempC.Receive -> AMReceiverC;
  LightTempC.AMSend -> AMSenderC;
  LightTempC.RadioControl -> ActiveMessageC;
  LightTempC.Packet -> AMSenderC;  

}

