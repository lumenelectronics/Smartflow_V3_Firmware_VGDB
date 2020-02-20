#include <Arduino.h>
#line 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
#line 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
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
#include <Lumen_SIM868.h>

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

#line 47 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
uint32_t Get_Flash_Checksum(uint32_t pStart, uint32_t pStop);
#line 62 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Setup_IO();
#line 110 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Service_Leds();
#line 201 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void LoRa_Setup();
#line 219 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Modem_Setup();
#line 240 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Flash_Setup();
#line 262 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Service_Switch();
#line 293 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void scan_i2c_bus();
#line 334 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void anISR();
#line 340 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void setup_counters();
#line 393 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void setup();
#line 427 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void loop();
#line 47 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
uint32_t Get_Flash_Checksum(uint32_t pStart, uint32_t pStop)
{
	uint32_t x = 0;
	volatile uint8_t *ptr = 0;
	volatile uint32_t check = 0;
	
	for (x = pStart;x < (pStop + 1);x++)
	{
		check += ptr[x];
	}
	
	return check;
}


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


bool ip_pulse=false;
bool ip_pulse_prev = false;

bool ip_alarm = false;
bool ip_alarm_prev = false;

bool ip_swt_cent = false;
bool ip_swt_cent_prev = false;


void Service_Switch()
{
/*	ip_pulse = !digitalRead(PIN_IP_PULSE);
	
	if (ip_pulse_prev != ip_pulse)
	{
		ip_pulse_prev = ip_pulse;
		DebugPort.print("ip_pulse=");
		DebugPort.println(ip_pulse);
	}*/
	
	ip_alarm = !digitalRead(PIN_IP_ALARM);
	if (ip_alarm_prev != ip_alarm)
	{
		ip_alarm_prev = ip_alarm;
		char Message[100];
		sprintf(Message, "ip_alarm=%d", (int)ip_alarm);
		DebugPort.println(Message);
		UsbPort.println(Message);		
	}
	
	ip_swt_cent = !digitalRead(PIN_SW_CENT);
	if (ip_swt_cent_prev != ip_swt_cent)
	{
		ip_swt_cent_prev = ip_swt_cent;
		char Message[100];
		sprintf(Message, "ip_swt_cent=%d", (int)ip_swt_cent);
		DebugPort.println(Message);
		UsbPort.println(Message);		
	}	
}
void scan_i2c_bus()
{
	byte error, address; //variable for error and I2C address
	int nDevices;

	UsbPort.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
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

void anISR()
{
	
}

// Adafruit Feather M4: Count input pulses with 32-bit timer TC0 on pin A4 and 30MHz test output with timer TCC0 on D10
void setup_counters()
{
	int InputPin = 50;	
	int InteruptNumber = 15;

 
	// EIC Interrupt System ///////////////////////////////////////////////////////////////////////////////
	//pinPeripheral(InputPin, PIO_EXTINT);
	
	attachInterrupt(digitalPinToInterrupt(InputPin), anISR, LOW);
	detachInterrupt(digitalPinToInterrupt(InputPin));	

	// Enable the port multiplexer on analog pin A4
	//PORT->Group[g_APinDescription[InputPin].ulPort].PINCFG[g_APinDescription[InputPin].ulPin].bit.PMUXEN = 1;
 
	// Set-up the pin as an EIC (interrupt) peripheral on analog pin InputPin
	//PORT->Group[g_APinDescription[InputPin].ulPort].PMUX[g_APinDescription[InputPin].ulPin >> 1].reg |= PORT_PMUX_PMUXE(0);	
 
	EIC->CTRLA.bit.ENABLE = 0;                        // Disable the EIC peripheral
	while(EIC->SYNCBUSY.bit.ENABLE);									// Wait for synchronization
	EIC->CONFIG[0].reg = EIC_CONFIG_SENSE4_LOW;       // Set event on detecting a HIGH level
	EIC->EVCTRL.reg = 1 << InteruptNumber;						// Enable event output on external interrupt 4
	EIC->INTENCLR.reg = 1 << InteruptNumber;					// Clear interrupt on external interrupt 4
	EIC->ASYNCH.reg = 1 << InteruptNumber;						// Set-up interrupt as asynchronous input
	EIC->CTRLA.bit.ENABLE = 1;                        // Enable the EIC peripheral
	while(EIC->SYNCBUSY.bit.ENABLE);									// Wait for synchronization

	// TC0 Count Timer //////////////////////////////////////////////////////////////////////////////////
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |           // Enable perhipheral channel for TC0
	                                 GCLK_PCHCTRL_GEN_GCLK0;     // Connect generic clock 0 at 120MHz

	TC0->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32;              // Set-up TC0/TC1 timers in 32-bit mode
 
	// Event System ///////////////////////////////////////////////////////////////////////////////
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;         // Switch on the event system peripheral
 
	// Select the event system user on channel 0 (USER number = channel number + 1)
	EVSYS->USER[EVSYS_ID_USER_TC0_EVU].reg = EVSYS_USER_CHANNEL(1);         // Set the event user (receiver) as timer TC0

	// Select the event system generator on channel 0
	EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
	EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
	EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_0 + InteruptNumber);   // Set event generator (sender) as external interrupt 4     

	TC0->COUNT32.EVCTRL.reg = TC_EVCTRL_TCEI |                // Enable the TC event input                       
	TC_EVCTRL_EVACT_COUNT;        // Set up the timer to count on event
 
	// Enable Timer  ///////////////////////////////////////////////////////////////////////////////
 
	TC0->COUNT32.CTRLA.bit.ENABLE = 1;                 // Enable timer TC0
	while(TC0->COUNT32.SYNCBUSY.bit.ENABLE);          // Wait for synchronization	
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
	
	//digitalWrite(PIN_VALVE_PWR_ON, HIGH);
	
	setup_counters();
	//test_loop();
}

uint32_t TC0_Cached=0;
bool TC0_Clear = 0;

void loop()
{	
	Output_Regs.Update();
	
	if (millis() > send_serial_tick)
	{				
		send_serial_tick = millis() + 1000;
		
		volatile uint16_t Vbat_raw = analogRead(PIN_BAT_AN);
		
		Vbat_raw = analogRead(PIN_BAT_AN);
		
		volatile uint32_t vbat_mv = Vbat_raw * 9668;
		vbat_mv /= 1000;
		
		{
			TC0->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_READSYNC;     // Initiate read synchronization of COUNT register
			while(TC0->COUNT32.SYNCBUSY.bit.CTRLB);                  // Wait for CTRLBSET register write synchronization
			while(TC0->COUNT32.SYNCBUSY.bit.COUNT);                  // Wait for COUNT register read synchronization		
			
			TC0_Cached = TC0->COUNT32.COUNT.reg;
		
			if (TC0_Clear)
			{
				TC0->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER;    // Initiate timer reset
				while(TC0->COUNT32.SYNCBUSY.bit.CTRLB);                  // Wait for CTRLBSET register write synchronization		
			}			
		}

		char TestMessage[100];
		snprintf(TestMessage, sizeof(TestMessage), "Serial USB,TC0:%ld,vbatt=%ld,millis()=%ld", TC0_Cached, vbat_mv, millis());
		UsbPort.println(TestMessage);		
		DebugPort.println(TestMessage);		
	}
	
	Service_Leds();		
  GSM_Modem.Service();
	Service_Switch();
}


