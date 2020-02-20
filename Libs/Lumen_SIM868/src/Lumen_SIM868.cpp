#include "Lumen_SIM868.h"

//TODO: add your code here

void SIM868_Unit::DebugString(char *temp)
{
	if (DebugPort)
	{
		DebugPort->println(temp);				
	}
}

void SIM868_Unit::DebugString(String temp)
{
	if (DebugPort)
	{
		DebugPort->print(DebugPrefix);				
		DebugPort->println(temp);				
	}
}

void SIM868_Unit::Send_Command_to_Modem(char *temp)
{
	GsmPort->println(temp);
}

void SIM868_Unit::Send_Command_to_Modem(String temp)
{
	GsmPort->println(temp);
}

void SIM868_Unit::Service()
{
	while (GsmPort->available())
	{		
		int temp_char = GsmPort->read();
		
		GsmRxBuffer += (char)temp_char;			
		
		if (temp_char == '\r')
		{
			GsmRxBuffer.replace("\n", "<LF>");
			GsmRxBuffer.replace("\r", "<CR>");
			DebugString(GsmRxBuffer);
			GsmRxBuffer = "";
		}		
	}
	
	if (!At_Echo_Done)
	{		
		if (millis() > 10000)
		{			
			Send_Command_to_Modem("AT");
			
			DebugString("Testing Modem for AT response");
			
			At_Echo_Done = true;			
		}
		
		return;
	}	
	
	if (millis() > check_reception_tick)
	{
		check_reception_tick = millis() + 5000;
		
		Send_Command_to_Modem("AT+CSQ");				
	}	
}

