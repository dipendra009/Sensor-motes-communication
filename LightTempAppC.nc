configuration LightTempAppC
{
}
implementation
{
  components LightTempC, MainC, LedsC, new TimerMilliC(), 
new HamamatsuS10871TsrC() as LightSensor;
  components new SensirionSht11C() as TempSensor;	
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components SerialPrintfC;
  

  LightTempC.Boot -> MainC;
  LightTempC.Timer0 -> Timer0;
  LightTempC.Timer1 -> Timer1;
  LightTempC.Light -> LightSensor;
  LightTempC.Temp -> TempSensor.Temperature;
  LightTempC.Leds -> LedsC;  

}

