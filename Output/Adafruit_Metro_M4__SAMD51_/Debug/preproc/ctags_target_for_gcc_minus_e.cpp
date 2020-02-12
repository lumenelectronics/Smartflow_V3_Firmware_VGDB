# 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino"
# 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino"
// ---------------------------------------------------------------- /
// Arduino I2C Scanner
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using GY-87 module for the target
// Tested on 10 September 2019
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
// ---------------------------------------------------------------- /

# 12 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 13 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 14 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 15 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 16 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2


// Serial Ports

// Serial = UsbPort
// Serial1 = SERCOM3 = (pin table index  0/1) = DATA_IN / DATA_OUT - DEBUG
// Serial0 = GSM mode SERCOM0
// I2C = SERCOM5 (pin table index  22/23)
// SPI = SERCOM2 (pin table index  24/25/26)
// FLASH_SPI = QSPI


Adafruit_SPIFlash flash(); // Use hardware SPI 
# 72 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino"
void Setup_IO()
{
 pinMode(14,(0x0));

 digitalWrite(36, (0x0));
 pinMode(36, (0x1));

 pinMode(49,(0x0));
 pinMode(27, (0x0));
 pinMode(29, (0x0));
 pinMode(17, (0x0));
 pinMode(15, (0x0));
 pinMode(16, (0x0));
 pinMode(47, (0x0));

 // valve inputs
 pinMode(40, (0x0));
 pinMode(51, (0x0));
 pinMode(9, (0x0));
 pinMode(8, (0x0));
 pinMode(3, (0x0));
 pinMode(2, (0x0));
 pinMode(10, (0x0));
 pinMode(11, (0x0));
 pinMode(13, (0x0));
 pinMode(12, (0x0));

 // spi bus + io to the lora module
 digitalWrite(53,(0x1));
 pinMode(26, (0x0));
 pinMode(27, (0x0));
 pinMode(28, (0x0));
 pinMode(53, (0x1));
 pinMode(54, (0x0)); // needs a tri-state to be idle!	

 // direct inputs
 pinMode(48, (0x0));
 pinMode(50, (0x0));
 pinMode(52, (0x0));
}



Uart Serial0(&sercom0, 0, 1, (SERCOM_RX_PAD_1), (UART_TX_PAD_0));

void SERCOM0_0_Handler()
{
 Serial0.IrqHandler();
}
void SERCOM0_1_Handler()
{
 Serial0.IrqHandler();
}
# 158 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_VGDB\\Smartflow_V3_VGDB\\sketches\\Smartflow_Metro_M4.ino"
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
    digitalWrite(pin_DS, (0x1));
   }
   else
   {
    digitalWrite(pin_DS, (0x0));
   }

   digitalWrite(pin_SH_CP, (0x1));
   temp_value <<= 1;
   digitalWrite(pin_SH_CP, (0x0));
  }

  digitalWrite(pin_ST_CP, (0x1));
  digitalWrite(pin_ST_CP, (0x0));

  UpdateTick = millis();
 }

public :
 Outputs_74HC595(int pDS,int pSH_CP, int pST_CP)
 {
  pin_DS = pDS;
  pin_SH_CP = pSH_CP;
  pin_ST_CP = pST_CP;

  pinMode(pin_DS, (0x1));
  pinMode(pin_SH_CP, (0x1));
  pinMode(pin_ST_CP, (0x1));

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


Outputs_74HC595 Output_Regs(7, 5/*4*/, 4/*5*/);

static long send_serial_tick;

void Service_Leds()
{
 static int pLeds;

 switch (pLeds)
 {
 case 0:
  {
   Output_Regs.Output_On(0x01000000);
   Output_Regs.Output_Off(0x08000000);
   Output_Regs.Output_Off(0x04000000);
   Output_Regs.Output_Off(0x02000000);
  }
  break;

 case 1:
  {
   Output_Regs.Output_On(0x01000000);
   Output_Regs.Output_On(0x08000000);
   Output_Regs.Output_Off(0x04000000);
   Output_Regs.Output_Off(0x02000000);
  }
  break;

 case 2:
  {
   Output_Regs.Output_Off(0x01000000);
   Output_Regs.Output_On(0x08000000);
   Output_Regs.Output_Off(0x04000000);
   Output_Regs.Output_Off(0x02000000);
  }
  break;

 case 3:
  {
   Output_Regs.Output_Off(0x01000000);
   Output_Regs.Output_On(0x08000000);
   Output_Regs.Output_On(0x04000000);
   Output_Regs.Output_Off(0x02000000);
  }
  break;

 case 4:
  {
   Output_Regs.Output_Off(0x01000000);
   Output_Regs.Output_Off(0x08000000);
   Output_Regs.Output_On(0x04000000);
   Output_Regs.Output_Off(0x02000000);
  }
  break;

 case 5:
  {
   Output_Regs.Output_Off(0x01000000);
   Output_Regs.Output_Off(0x08000000);
   Output_Regs.Output_On(0x04000000);
   Output_Regs.Output_On(0x02000000);
  }
  break;

 case 6:
  {
   Output_Regs.Output_Off(0x01000000);
   Output_Regs.Output_Off(0x08000000);
   Output_Regs.Output_Off(0x04000000);
   Output_Regs.Output_On(0x02000000);
  }
  break;

 case 7:
  {
   Output_Regs.Output_On(0x01000000);
   Output_Regs.Output_Off(0x08000000);
   Output_Regs.Output_Off(0x04000000);
   Output_Regs.Output_On(0x02000000);
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
 Serial.println("Smartflow Io test");

 Serial0.begin(115200);
 // Assign pins 10 & 11 SERCOM functionality
 pinPeripheral(1, PIO_SERCOM);
 pinPeripheral(0, PIO_SERCOM);
 Serial0.println("Smartflow Io test");

 Serial1.begin(115200);
 Serial1.println("Smartflow Io test");

 Output_Regs.Output_On(0x00000400);
 Output_Regs.Output_On(0x00000800);

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

  volatile uint16_t Vbat_raw = analogRead(14);

  Vbat_raw = analogRead(14);

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
      Serial.print(address, 16);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, 16);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}
