# 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
# 1 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
# 2 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
// ---------------------------------------------------------------- /
// Arduino I2C Scanner
// Re-writed by Arbi Abdul Jabbaar
// Using Arduino IDE 1.8.7
// Using GY-87 module for the target
// Tested on 10 September 2019
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
// ---------------------------------------------------------------- /

# 13 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 14 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 15 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 16 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2
# 17 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2


# 20 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2

# 22 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2


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


Adafruit_FlashTransport_QSPI flashTransport((41u), (42u), (43u), (44u), (45u), (46u));
Adafruit_SPIFlash flash_chip(&flashTransport);




void Setup_IO()
{
 pinMode(14,(0x0));

 digitalWrite(36, (0x0));
 pinMode(36, (0x1));

 pinMode(49,(0x0));
 pinMode(27, (0x0));
 pinMode(29, (0x0));

 pinMode((55ul), (0x1));
 digitalWrite((55ul), (0x1));

 digitalWrite(47, (0x0));
 pinMode(47, (0x1));
 pinMode(16, (0x0));

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

Outputs_74HC595 Output_Regs(7, 5/*4*/, 4/*5*/);

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

 Output_Regs.Output_On(0x00000800);
 Output_Regs.Output_On(0x10000000);
 delay(1000);

 Output_Regs.Output_On(0x00000400);
 Output_Regs.Output_On(0x20000000);
 delay(1000);
 Output_Regs.Output_Off(0x00000400);
 Output_Regs.Output_On(0x40000000);

 delay(1000);

 Output_Regs.Output_Off(0x10000000);
 Output_Regs.Output_Off(0x20000000);
 Output_Regs.Output_Off(0x40000000);
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
 UsbPort.println("Smartflow V3");
 UsbPort.flush();

 DebugPort.begin(115200);
 DebugPort.println("Smartflow V3");
 DebugPort.flush();

 GsmPort.begin(115200);
 GsmPort.println("Smartflow V3");
 GsmPort.flush();

 LoRa_Setup();

 Modem_Setup();

 Flash_Setup();
}

bool ip_pulse=false;
bool ip_pulse_prev = false;

bool ip_alarm = false;
bool ip_alarm_prev = false;

bool ip_swt_cent = false;
bool ip_swt_cent_prev = false;


void Service_Switch()
{
 ip_pulse = !digitalRead(50);

 if (ip_pulse_prev != ip_pulse)
 {
  ip_pulse_prev = ip_pulse;
  DebugPort.print("ip_pulse=");
  DebugPort.println(ip_pulse);
 }

 ip_alarm = !digitalRead(48);
 if (ip_alarm_prev != ip_alarm)
 {
  ip_alarm_prev = ip_alarm;
  DebugPort.print("ip_alarm=");
  DebugPort.println(ip_alarm);
 }

 ip_swt_cent = !digitalRead(52);
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

  volatile uint16_t Vbat_raw = analogRead(14);

  Vbat_raw = analogRead(14);

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
      UsbPort.print(address, 16);
      UsbPort.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      UsbPort.print("Unknown error at address 0x");
      if (address < 16)
        UsbPort.print("0");
      UsbPort.println(address, 16);
    }
  }
  if (nDevices == 0)
    UsbPort.println("No I2C devices found\n");
  else
    UsbPort.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}
