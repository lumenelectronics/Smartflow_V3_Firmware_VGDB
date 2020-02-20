#pragma once

#include <Arduino.h>

//TODO: add your function declarations here
class SIM868_Unit
{
	long check_reception_tick=0;
	const long check_reception_interval = 5000;
	bool At_Echo_Done=false;
	
	void Send_Command_to_Modem(char *temp);
	void Send_Command_to_Modem(String temp);
	void DebugString(char *temp);
	void DebugString(String temp);
	
	String DebugPrefix;	
	
	Stream *GsmPort;
	Stream *DebugPort;
	
	String GsmRxBuffer;

public:
	SIM868_Unit(Stream *pGsmPort, Stream *pDebugPort,String pDebugPrefix)
	{
		GsmPort = pGsmPort;
		DebugPort = pDebugPort;
		DebugPrefix = pDebugPrefix;
	}
	
	void Service();	
};