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

# 18 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2


# 21 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2

# 23 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino" 2


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
# 260 "D:\\SourceTree\\Smartflow\\Firmware\\Smartflow_V3_Firmware_VGDB\\sketches\\Smartflow_Metro_M4.ino"
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

 attachInterrupt(( InputPin ), anISR, (0x0));
 detachInterrupt(( InputPin ));

 // Enable the port multiplexer on analog pin A4
 //PORT->Group[g_APinDescription[InputPin].ulPort].PINCFG[g_APinDescription[InputPin].ulPin].bit.PMUXEN = 1;

 // Set-up the pin as an EIC (interrupt) peripheral on analog pin InputPin
 //PORT->Group[g_APinDescription[InputPin].ulPort].PMUX[g_APinDescription[InputPin].ulPin >> 1].reg |= PORT_PMUX_PMUXE(0);	

 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->CTRLA.bit.ENABLE = 0; // Disable the EIC peripheral
 while(((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->SYNCBUSY.bit.ENABLE); // Wait for synchronization
 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->CONFIG[0].reg = (0x5U /**< C code: Unsigned integer literal constant value */ /**< \brief (EIC_CONFIG) Low level detection */ << 16 /**< \brief (EIC_CONFIG) Input Sense Configuration 4 */); // Set event on detecting a HIGH level
 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->EVCTRL.reg = 1 << InteruptNumber; // Enable event output on external interrupt 4
 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->INTENCLR.reg = 1 << InteruptNumber; // Clear interrupt on external interrupt 4
 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->ASYNCH.reg = 1 << InteruptNumber; // Set-up interrupt as asynchronous input
 ((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->CTRLA.bit.ENABLE = 1; // Enable the EIC peripheral
 while(((Eic *)0x40002800UL) /**< \brief (EIC) APB Base Address */->SYNCBUSY.bit.ENABLE); // Wait for synchronization

 // TC0 Count Timer //////////////////////////////////////////////////////////////////////////////////
 ((Gclk *)0x40001C00UL) /**< \brief (GCLK) APB Base Address */->PCHCTRL[9 /* Index of Generic Clock*/].reg = (0x1U /**< C code: Unsigned integer literal constant value */ << 6 /**< \brief (GCLK_PCHCTRL) Channel Enable */) | // Enable perhipheral channel for TC0
                                  (0x0U /**< C code: Unsigned integer literal constant value */ /**< \brief (GCLK_PCHCTRL) Generic clock generator 0 */ << 0 /**< \brief (GCLK_PCHCTRL) Generic Clock Generator */); // Connect generic clock 0 at 120MHz

 ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.CTRLA.reg = (0x2U /**< C code: Unsigned integer literal constant value */ /**< \brief (TC_CTRLA) Counter in 32-bit mode */ << 2 /**< \brief (TC_CTRLA) Timer Counter Mode */); // Set-up TC0/TC1 timers in 32-bit mode

 // Event System ///////////////////////////////////////////////////////////////////////////////
 ((Mclk *)0x40000800UL) /**< \brief (MCLK) APB Base Address */->APBBMASK.reg |= (0x1U /**< C code: Unsigned integer literal constant value */ << 7 /**< \brief (MCLK_APBBMASK) EVSYS APB Clock Enable */); // Switch on the event system peripheral

 // Select the event system user on channel 0 (USER number = channel number + 1)
 ((Evsys *)0x4100E000UL) /**< \brief (EVSYS) APB Base Address */->USER[44].reg = ((0x3FU /**< C code: Unsigned integer literal constant value */ << 0 /**< \brief (EVSYS_USER) Channel Event Selection */) & ((1) << 0 /**< \brief (EVSYS_USER) Channel Event Selection */)); // Set the event user (receiver) as timer TC0

 // Select the event system generator on channel 0
 ((Evsys *)0x4100E000UL) /**< \brief (EVSYS) APB Base Address */->Channel[0].CHANNEL.reg = (0x0U /**< C code: Unsigned integer literal constant value */ /**< \brief (EVSYS_CHANNEL) No event output when using the resynchronized or synchronous path */ << 10 /**< \brief (EVSYS_CHANNEL) Edge Detection Selection */) | // No event edge detection
 (0x2U /**< C code: Unsigned integer literal constant value */ /**< \brief (EVSYS_CHANNEL) Asynchronous path */ << 8 /**< \brief (EVSYS_CHANNEL) Path Selection */) | // Set event path as asynchronous
 ((0x7FU /**< C code: Unsigned integer literal constant value */ << 0 /**< \brief (EVSYS_CHANNEL) Event Generator Selection */) & ((18 + InteruptNumber) << 0 /**< \brief (EVSYS_CHANNEL) Event Generator Selection */)); // Set event generator (sender) as external interrupt 4     

 ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.EVCTRL.reg = (0x1U /**< C code: Unsigned integer literal constant value */ << 5 /**< \brief (TC_EVCTRL) TC Event Enable */) | // Enable the TC event input                       
 (0x2U /**< C code: Unsigned integer literal constant value */ /**< \brief (TC_EVCTRL) Count on event */ << 0 /**< \brief (TC_EVCTRL) Event Action */); // Set up the timer to count on event

 // Enable Timer  ///////////////////////////////////////////////////////////////////////////////

 ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.CTRLA.bit.ENABLE = 1; // Enable timer TC0
 while(((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.SYNCBUSY.bit.ENABLE); // Wait for synchronization	
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

  volatile uint16_t Vbat_raw = analogRead(14);

  Vbat_raw = analogRead(14);

  volatile uint32_t vbat_mv = Vbat_raw * 9668;
  vbat_mv /= 1000;

  {
   ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.CTRLBSET.reg = (0x4U /**< C code: Unsigned integer literal constant value */ /**< \brief (TC_CTRLBCLR) Force a read synchronization of COUNT */ << 5 /**< \brief (TC_CTRLBCLR) Command */); // Initiate read synchronization of COUNT register
   while(((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.SYNCBUSY.bit.CTRLB); // Wait for CTRLBSET register write synchronization
   while(((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.SYNCBUSY.bit.COUNT); // Wait for COUNT register read synchronization		

   TC0_Cached = ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.COUNT.reg;

   if (TC0_Clear)
   {
    ((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.CTRLBSET.reg = (0x1U /**< C code: Unsigned integer literal constant value */ /**< \brief (TC_CTRLBCLR) Force a start, restart or retrigger */ << 5 /**< \brief (TC_CTRLBCLR) Command */); // Initiate timer reset
    while(((Tc *)0x40003800UL) /**< \brief (TC0) APB Base Address */->COUNT32.SYNCBUSY.bit.CTRLB); // Wait for CTRLBSET register write synchronization		
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
