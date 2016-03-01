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
	
	App.Boot -> MainC;
	App.Leds -> LedsC;
	
	//for writing into serial port
	components SerialPrintfC;
	
	//Radio Communication
	components ActiveMessageC;
	components new AMReceiverC(AM_RADIO);
	

	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;
	
}