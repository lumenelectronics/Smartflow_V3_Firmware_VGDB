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


// Serial Ports

// Serial = UsbPort
// Serial1 = SERCOM3 = (pin table index  0/1) = DATA_IN / DATA_OUT - DEBUG
// Serial0 = GSM mode SERCOM0
// I2C = SERCOM5 (pin table index  22/23)
// SPI = SERCOM2 (pin table index  24/25/26)
// FLASH_SPI = QSPI


Adafruit_SPIFlash flash(EXTERNAL_FLASH_USE_QSPI);     // Use hardware SPI 

#define BOOT_MESSAGE "Smartflow Io test"

#define PIN_BAT_AN 14
#define PIN_VALVE_PWR_ON 36
#define PIN_IP_ALARM 48
#define PIN_SAMD51_NETLIGHT 49

#define PIN_SAMD51_STATUS 27
#define PIN_SAMD51_RI 29

#define PIN_SAMD51_TX 17
#define PIN_SAMD51_RX 15
#define PIN_SAMD51_RTS 16
#define PIN_SAMD51_CTS 47

#define PIN_DS 7
#define PIN_SH_CP 5//4
#define PIN_ST_CP 4//5

#define PIN_MOSI 26
#define PIN_SCK 27
#define PIN_MISO 28
#define PIN_IP_PULSE 50

#define PIN_V5_CL 13
#define PIN_V5_OP 12
#define PIN_V4_CL 10
#define PIN_V4_OP 11
#define PIN_V3_CL 3
#define PIN_V3_OP 2
#define PIN_V2_CL 9
#define PIN_V2_OP 8
#define PIN_DATA_OUT 1
#define PIN_DATA_IN 0

#define PIN_V1_CL 40
#define PIN_V1_OP 51

#define PIN_SW_CENT 52
#define PIN_LORA_CS 53
#define PIN_LORA_RESET 54

void Setup_IO()
{
	pinMode(PIN_BAT_AN,INPUT);
	
	digitalWrite(PIN_VALVE_PWR_ON, LOW);
	pinMode(PIN_VALVE_PWR_ON, OUTPUT);

	pinMode(PIN_SAMD51_NETLIGHT,INPUT);
	pinMode(PIN_SAMD51_STATUS, INPUT);
	pinMode(PIN_SAMD51_RI, INPUT);
	pinMode(PIN_SAMD51_TX, INPUT);
	pinMode(PIN_SAMD51_RX, INPUT);
	pinMode(PIN_SAMD51_RTS, INPUT);
	pinMode(PIN_SAMD51_CTS, INPUT);
	
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



Uart Serial0(&sercom0, PIN_DATA_IN, PIN_DATA_OUT, PAD_SERIAL1_RX, PAD_SERIAL1_TX);

void SERCOM0_0_Handler()
{
	Serial0.IrqHandler();
}
void SERCOM0_1_Handler()
{
	Serial0.IrqHandler();
}

#define SO_V3_OPEN				0x00000002
#define SO_V3_CLOSE				0x00000004
#define SO_V2_OPEN				0x00000008
#define SO_V2_CLOSE				0x00000010
#define SO_V1_OPEN				0x00000020
#define SO_V1_CLOSE				0x00000040

#define SO_GSM_SW					0x00000400
#define SO_GSM_PWR				0x00000800
#define SO_V5_OPEN				0x00001000
#define SO_V5_CLOSE				0x00002000
#define SO_V4_OPEN				0x00004000
#define SO_V4_CLOSE				0x00008000
#define LED_REMOTE				0x00010000
#define LED_GSM						0x00020000
#define LED_VALVE_5				0x00040000
#define LED_VALVE_4				0x00080000
#define LED_VALVE_3				0x00100000
#define LED_VALVE_2				0x00200000
#define LED_VALVE_1				0x00400000

#define LED_MAIN_TOP			0x01000000
#define LED_MAIN_LEFT			0x02000000
#define LED_MAIN_BOTTOM		0x04000000
#define LED_MAIN_RIGHT		0x08000000
#define LED_DIAG_1				0x10000000
#define LED_DIAG_2				0x20000000
#define LED_DIAG_3				0x40000000
#define LED_DIAG_4				0x80000000


	
class Outputs_74HC595
{	
	uint32_t OutputBits;
	
	int pin_DS;
	int pin_SH_CP;
	int pin_ST_CP;
	
	uint32_t UpdateTick;
	
	void Shift_Data()
	{
		// q0 is first flip flop in the chain so we must end with bit 0 done last
		volatile uint32_t temp_value = OutputBits;
	
		for (int x = 0;x < 32;x++)
		{
			if ((temp_value & 0x80000000) == 0x80000000)
			{
				digitalWrite(pin_DS, HIGH);
			}
			else
			{
				digitalWrite(pin_DS, LOW);
			}
		
			digitalWrite(pin_SH_CP, HIGH);
			temp_value <<= 1;		
			digitalWrite(pin_SH_CP, LOW);
		}		
	
		digitalWrite(pin_ST_CP, HIGH);
		digitalWrite(pin_ST_CP, LOW);		
		
		UpdateTick = millis();
	}
		
public :		
	Outputs_74HC595(int pDS,int pSH_CP, int pST_CP)
	{	
		pin_DS = pDS;
		pin_SH_CP = pSH_CP;
		pin_ST_CP = pST_CP;		
		
		pinMode(pin_DS, OUTPUT);
		pinMode(pin_SH_CP, OUTPUT);
		pinMode(pin_ST_CP, OUTPUT);
		
		OutputBits = 0;
		
		Shift_Data();
	}

public:		
	void Update()
	{
		if ((millis() - UpdateTick) > 100)
		{
			Shift_Data();
		}						
	}	
	
	void Output_On(int pMask)
	{
		OutputBits |= pMask;
	}	

	void Output_Off(int pMask)
	{
		OutputBits &= ~pMask;		
	}	
};


Outputs_74HC595 Output_Regs(PIN_DS, PIN_SH_CP, PIN_ST_CP);

static long send_serial_tick;

void Service_Leds()
{
	static int pLeds;
		
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

void setup()
{	
	Setup_IO();
	Wire.begin(); // Wire communication begin
	SPI.begin();
	
	Serial.begin(115200);
	Serial.println(BOOT_MESSAGE);
	
	Serial0.begin(115200);
	// Assign pins 10 & 11 SERCOM functionality
	pinPeripheral(PIN_DATA_OUT, PIO_SERCOM);
	pinPeripheral(PIN_DATA_IN, PIO_SERCOM);	
	Serial0.println(BOOT_MESSAGE);
	
	Serial1.begin(115200);	
	Serial1.println(BOOT_MESSAGE);		
	
	Output_Regs.Output_On(SO_GSM_SW);
	Output_Regs.Output_On(SO_GSM_PWR);	
	
	Serial1.println("LoRa Receiver");
	
	if (!LoRa.begin(915E6))
	{
		Serial1.println("Starting LoRa failed!");
	}	
	else
	{
		Serial1.println("Starting LoRa OK");
	}
	
	LoRa.dumpRegisters(Serial1);
	
	Serial1.flush();
}

void loop()
{	
	Output_Regs.Update();
	
	if (millis() > send_serial_tick)
	{		
		send_serial_tick = millis() + 200;
		
		volatile uint16_t Vbat_raw = analogRead(PIN_BAT_AN);
		
		Vbat_raw = analogRead(PIN_BAT_AN);
		
		volatile uint32_t vbat_mv = Vbat_raw * 9668;
		vbat_mv /= 1000;
		
		char TestMessage[100];
		snprintf(TestMessage, sizeof(TestMessage), "Serial USB,vbat=%ld,millis()=%ld", vbat_mv ,millis());
		Serial.println(TestMessage);
		
		snprintf(TestMessage, sizeof(TestMessage), "Serial0 GSM,vbat=%ld,millis()=%ld", vbat_mv, millis());
		Serial0.println(TestMessage);
		
		snprintf(TestMessage, sizeof(TestMessage), "Serial1 DEBUG,vbat=%ld,millis()=%ld", vbat_mv, millis());
		Serial1.println(TestMessage);	
		
		Service_Leds();		
	}
}

void scan_i2c_bus()
{
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

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
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}
