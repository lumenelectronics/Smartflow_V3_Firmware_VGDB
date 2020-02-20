#include <Arduino.h>

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
		Shift_Data();
	}	

	void Output_Off(int pMask)
	{
		OutputBits &= ~pMask;		
		Shift_Data();
	}	
};
