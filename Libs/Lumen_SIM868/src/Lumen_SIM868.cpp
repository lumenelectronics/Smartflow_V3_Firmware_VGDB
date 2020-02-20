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

String Ftp_Fetch_Dat_List[] = 
{
	"AT",
	"ATI",
	"AT+GSV",
	"AT+CSQ",
	"AT+CIMI",
	"AT+GSN",
	"AT+CPIN",
	"AT+CGATT?",
	"AT+SAPBR=3,1,\"Contype\",\"GPRS\"",
	"AT+SAPBR=3,1,\"APN\",\"ESEYE1\"",
	"AT+SAPBR=3,1,\"USER\",\"USER\"",
	"AT+SAPBR=3,1,\"PWD\",\"PASS\"",
	"AT+SAPBR=1,1",
	"AT+SAPBR=2,1",
	"AT+FTPCID=1",
	"AT+FTPPORT=21",
	"AT+FTPSERV=\"lumenelectronics.ddns.net\"",
	"AT+FTPUN=\"gunjan-ftp\"",
	"AT+FTPPW=\"lumen\"",
	"AT+FTPMODE=0",
	"AT+FTPTYPE=I",
	"AT+FTPGETNAME=\"smarflow3v00.dat\"",
	"AT+FTPGETPATH=\"/\"",
	"AT+FTPSTATE",
	"AT+FTPGET=1",		
};

void SIM868_Unit::FtpService()
{		
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

