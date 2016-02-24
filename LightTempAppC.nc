configuration LightTempAppC
{
}
implementation
{
  components LightTempC as App, MainC, LedsC, new TimerMilliC(), new HamamatsuS10871TsrC() as LightSensor;
  components new SensirionSht11C() as TempSensor;	
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new TimerMilliC() as Timer2;
  components SerialPrintfC;
  components new AMSenderC(AM_RADIO_COUNT_MSG);
  components new AMReceiverC(AM_RADIO_COUNT_MSG);
  components new ActiveMessageC;

  App.Boot -> MainC;
  App.Timer0 -> Timer0;
  App.Timer1 -> Timer1;
  App.Light -> LightSensor;
  App.Temp -> TempSensor.Temperature;
  App.Leds -> LedsC;  
  App.Receive -> AMReceiverC;
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.Packet -> AMSenderC;

}

