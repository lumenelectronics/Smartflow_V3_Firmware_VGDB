#include <Arduino.h>
#line 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
#line 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
#include <Lumen_SIM8686.h>
// ---------------------------------------------------------------- /
// Arduino I2C Scanner
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using GY-87 module for the target
// Tested on 10 September 2019
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
// ---------------------------------------------------------------- /

#include <Wire.h> //include Wire.h library
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include "wiring_private.h" // pinPeripheral() function
#include <LoRa.h>
#include <Lumen_SIM8686.h>

#include "Smartflow_V3_IO.h"

#include "Smartflow_74HC595.h"


// Serial Ports

// Serial = UsbPort
// Serial1 = SERCOM3 = (pin table index  0/1) = DATA_IN / DATA_OUT - DEBUG
// Serial0 = GSM mode SERCOM0
// I2C = SERCOM5 (pin table index  22/23)
// SPI = SERCOM2 (pin table index  24/25/26)
// FLASH_SPI = QSPI

Uart& DebugPort=Serial1;

Uart& GsmPort=Serial0;

Serial_& UsbPort=Serial;

SIM868_Unit GSM_Modem((Stream*)&GsmPort, (Stream*) &DebugPort, "GSM>>");


Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
Adafruit_SPIFlash flash_chip(&flashTransport);

#define BOOT_MESSAGE "Smartflow V3"


#line 48 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Setup_IO();
#line 96 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Service_Leds();
#line 187 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void LoRa_Setup();
#line 205 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Modem_Setup();
#line 226 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Flash_Setup();
#line 237 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void setup();
#line 275 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Service_Switch();
#line 303 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void loop();
#line 328 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void scan_i2c_bus();
#line 48 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Setup_IO()
{
	pinMode(PIN_BAT_AN,INPUT);
	
	digitalWrite(PIN_VALVE_PWR_ON, LOW);
	pinMode(PIN_VALVE_PWR_ON, OUTPUT);

	pinMode(PIN_SAMD51_NETLIGHT,INPUT);
	pinMode(PIN_SAMD51_STATUS, INPUT);
	pinMode(PIN_SAMD51_RI, INPUT);
	
	pinMode(PIN_SERIAL0_TX, OUTPUT);
	digitalWrite(PIN_SERIAL0_TX, HIGH);	
	
	digitalWrite(PIN_SAMD51_CTS, LOW);
	pinMode(PIN_SAMD51_CTS, OUTPUT);
	pinMode(PIN_SAMD51_RTS, INPUT);
	
	// valve inputs
	pinMode(PIN_V1_CL, INPUT);
	pinMode(PIN_V1_OP, INPUT);
	pinMode(PIN_V2_CL, INPUT);
	pinMode(PIN_V2_OP, INPUT);
	pinMode(PIN_V3_CL, INPUT);
	pinMode(PIN_V3_OP, INPUT);
	pinMode(PIN_V4_CL, INPUT);
	pinMode(PIN_V4_OP, INPUT);
	pinMode(PIN_V5_CL, INPUT);
	pinMode(PIN_V5_OP, INPUT);		

	// spi bus + io to the lora module
	digitalWrite(PIN_LORA_CS,HIGH);
	pinMode(PIN_MOSI, INPUT);
	pinMode(PIN_SCK, INPUT);
	pinMode(PIN_MISO, INPUT);
	pinMode(PIN_LORA_CS, OUTPUT);
	pinMode(PIN_LORA_RESET, INPUT); // needs a tri-state to be idle!	

	// direct inputs
	pinMode(PIN_IP_ALARM, INPUT);
	pinMode(PIN_IP_PULSE, INPUT);	
	pinMode(PIN_SW_CENT, INPUT);	
}
	
Outputs_74HC595 Output_Regs(PIN_DS, PIN_SH_CP, PIN_ST_CP);

static long send_serial_tick;

void Service_Leds()
{
	static int pLeds;	
	static long led_service_tick;
	
	if (millis() < led_service_tick)
	{
		return;
	}
	
	led_service_tick = millis() + 200;
		
	switch (pLeds)
	{
	case 0:
		{
			Output_Regs.Output_On(LED_MAIN_TOP);
			Output_Regs.Output_Off(LED_MAIN_RIGHT);
			Output_Regs.Output_Off(LED_MAIN_BOTTOM);
			Output_Regs.Output_Off(LED_MAIN_LEFT);
		}
		break;
						
	case 1:
		{
			Output_Regs.Output_On(LED_MAIN_TOP);
			Output_Regs.Output_On(LED_MAIN_RIGHT);
			Output_Regs.Output_Off(LED_MAIN_BOTTOM);
			Output_Regs.Output_Off(LED_MAIN_LEFT);
		}
		break;

	case 2:
		{
			Output_Regs.Output_Off(LED_MAIN_TOP);
			Output_Regs.Output_On(LED_MAIN_RIGHT);
			Output_Regs.Output_Off(LED_MAIN_BOTTOM);
			Output_Regs.Output_Off(LED_MAIN_LEFT);				
		}
		break;

	case 3:
		{
			Output_Regs.Output_Off(LED_MAIN_TOP);
			Output_Regs.Output_On(LED_MAIN_RIGHT);
			Output_Regs.Output_On(LED_MAIN_BOTTOM);
			Output_Regs.Output_Off(LED_MAIN_LEFT);				
		}
		break;

	case 4:
		{
			Output_Regs.Output_Off(LED_MAIN_TOP);
			Output_Regs.Output_Off(LED_MAIN_RIGHT);
			Output_Regs.Output_On(LED_MAIN_BOTTOM);
			Output_Regs.Output_Off(LED_MAIN_LEFT);
		}
		break;
						
	case 5:
		{
			Output_Regs.Output_Off(LED_MAIN_TOP);
			Output_Regs.Output_Off(LED_MAIN_RIGHT);
			Output_Regs.Output_On(LED_MAIN_BOTTOM);
			Output_Regs.Output_On(LED_MAIN_LEFT);
		}
		break;

	case 6:
		{
			Output_Regs.Output_Off(LED_MAIN_TOP);
			Output_Regs.Output_Off(LED_MAIN_RIGHT);
			Output_Regs.Output_Off(LED_MAIN_BOTTOM);
			Output_Regs.Output_On(LED_MAIN_LEFT);				
		}
		break;

	case 7:
		{
			Output_Regs.Output_On(LED_MAIN_TOP);
			Output_Regs.Output_Off(LED_MAIN_RIGHT);
			Output_Regs.Output_Off(LED_MAIN_BOTTOM);
			Output_Regs.Output_On(LED_MAIN_LEFT);				
		}
		break;
	}
		
	pLeds++;
	pLeds %= 8;	
}

void LoRa_Setup()
{
	DebugPort.println("LoRa Receiver");
	
	if (!LoRa.begin(915E6))
	{
		DebugPort.println("Starting LoRa failed!");
	}	
	else
	{
		DebugPort.println("Starting LoRa OK");
	}
	
	LoRa.dumpRegisters(DebugPort);	
	
	DebugPort.flush();
}

void Modem_Setup()
{
	DebugPort.println("Setting Up Modem");
	
	Output_Regs.Output_On(SO_GSM_PWR);
	Output_Regs.Output_On(LED_DIAG_1);
	delay(1000);
	
	Output_Regs.Output_On(SO_GSM_SW);
	Output_Regs.Output_On(LED_DIAG_2);	
	delay(1000);
	Output_Regs.Output_Off(SO_GSM_SW);
	Output_Regs.Output_On(LED_DIAG_3);
	
	delay(1000);
	
	Output_Regs.Output_Off(LED_DIAG_1);
	Output_Regs.Output_Off(LED_DIAG_2);
	Output_Regs.Output_Off(LED_DIAG_3);	
}

void Flash_Setup()
{
	flash_chip.begin();
	
	char temp[100];
	snprintf(temp, sizeof(temp), "flash.getJEDECID()=%08lX", flash_chip.getJEDECID());
	
	DebugPort.println(temp);
	DebugPort.flush();
}

void setup()
{	
	Setup_IO();
	Wire.begin(); // Wire communication begin
	SPI.begin();
	

	UsbPort.begin(115200);
	UsbPort.println(BOOT_MESSAGE);
	UsbPort.flush();
		
	DebugPort.begin(115200);	
	DebugPort.println(BOOT_MESSAGE);
	DebugPort.flush();
	
	GsmPort.begin(115200);	
	GsmPort.println(BOOT_MESSAGE);
	GsmPort.flush();
	
	LoRa_Setup();
	
	Modem_Setup();		
	
	Flash_Setup();		
	
	digitalWrite(PIN_VALVE_PWR_ON, HIGH);
}

bool ip_pulse=false;
bool ip_pulse_prev = false;

bool ip_alarm = false;
bool ip_alarm_prev = false;

bool ip_swt_cent = false;
bool ip_swt_cent_prev = false;


void Service_Switch()
{
	ip_pulse = !digitalRead(PIN_IP_PULSE);
	
	if (ip_pulse_prev != ip_pulse)
	{
		ip_pulse_prev = ip_pulse;
		DebugPort.print("ip_pulse=");
		DebugPort.println(ip_pulse);
	}
	
	ip_alarm = !digitalRead(PIN_IP_ALARM);
	if (ip_alarm_prev != ip_alarm)
	{
		ip_alarm_prev = ip_alarm;
		DebugPort.print("ip_alarm=");
		DebugPort.println(ip_alarm);
	}
	
	ip_swt_cent = !digitalRead(PIN_SW_CENT);
	if (ip_swt_cent_prev != ip_swt_cent)
	{
		ip_swt_cent_prev = ip_swt_cent;
		DebugPort.print("ip_swt_cent=");
		DebugPort.println(ip_swt_cent);
	}	
}

void loop()
{	
	Output_Regs.Update();
	
	if (millis() > send_serial_tick)
	{				
		send_serial_tick = millis() + 100;
		
		volatile uint16_t Vbat_raw = analogRead(PIN_BAT_AN);
		
		Vbat_raw = analogRead(PIN_BAT_AN);
		
		volatile uint32_t vbat_mv = Vbat_raw * 9668;
		vbat_mv /= 1000;
		
		char TestMessage[100];
		snprintf(TestMessage, sizeof(TestMessage), "Serial USB,vbatt=%ld,millis()=%ld", vbat_mv ,millis());
		UsbPort.println(TestMessage);				
	}
	
	Service_Leds();		
  GSM_Modem.Service();
	Service_Switch();
}

void scan_i2c_bus()
{
  byte error, address; //variable for error and I2C address
  int nDevices;

  UsbPort.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      UsbPort.print("I2C device found at address 0x");
      if (address < 16)
        UsbPort.print("0");
      UsbPort.print(address, HEX);
      UsbPort.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      UsbPort.print("Unknown error at address 0x");
      if (address < 16)
        UsbPort.print("0");
      UsbPort.println(address, HEX);
    }
  }
  if (nDevices == 0)
    UsbPort.println("No I2C devices found\n");
  else
    UsbPort.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}

