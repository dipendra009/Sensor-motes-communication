configuration TwoTelosbAppC
{
	//Not interested now!
}

implementation
{
	//General
	components TwoTelosbC as App; //Main module file
	components MainC; //Boot
	components LedsC; //Leds
	components new TimerMilliC() as Timer0;
	components new TimerMilliC() as Timer1;
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.Timer0 -> Timer0;
	App.Timer1 -> Timer1;
	
	//for writing into serial port
	components SerialPrintfC;
	
	//Temperature component
	components new SensirionSht11C() as TempSensor;
	
	App.TempRead -> TempSensor.Temperature;
	
	//Light component 
	components new HamamatsuS10871TsrC() as LightSensor;
	
	App.LightRead -> LightSensor;
	
	//Humidity component
	
	App.HumidRead -> TempSensor.Humidity;
	
	//Radio Communication
	components ActiveMessageC;
	components new AMSenderC(AM_RADIO);
	//components new AMReceiverC(AM_RADIO);
	
	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
	App.AMSend -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	//App.Receive -> AMReceiverC;
	
}