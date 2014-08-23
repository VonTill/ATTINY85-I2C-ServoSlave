/*
/	ATTINY 85 I2C(TWI) to Dual Servo Controller
/	
/	Using only Timer/Counter1: Outputs are PB1(OC1A) and PB4(OC1B)
/	Using PB3 as Output ( but can be either input or also ADC)
/  	
/
/	Read all notes, I have tried to be as descriptive as possible, 
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "TinyWireS.h"
//#include <delay.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define I2C_SLAVE_ADDR  0x2F

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint8_t u8PwmA     = 0;	// Center Value for OCR1A
uint8_t u8PwmB     = 0;	// Center Value for OCR1B
uint8_t u8Command  = 0;
uint8_t u8PB3	   = 0; // output or input on PB3 variable

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void initPWM() {
  
  DDRB  |= (1 << PB1) | // OC0B  :  PB1  :  Pin 6  Set as Output
	  (1 << PB4) ;		// OC1B  :  PB4  :  Pin 3  Set as Output
  // OC0A cannot be used, because PB0 = pin 5 is used by I2C (SDA),
  // only use Timer/Counter1
  // Timer/Conter0 setup is here if you want to alter it.
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //                 Timer / Counter 0 Setup
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  TCCR0A = // ------------ Timer/Counter0 Control Register A ------------
      (0 << COM0A1) | (0 << COM0A0) |   //  Compare Match Output Mode OCR0A : PB0 :		DISCONNECTED
      (0 << COM0B1) | (0 << COM0B0) |   //  Compare Match Output Mode OCR0B : PB1 :		DISCONNECTED
      (0 << WGM01)  | (0 << WGM00);     //  Waveform Genration Bits 1-0 :  
  
  TCCR0B = // ------------ Timer/Counter0 Control Register B ------------
      (0 << FOC0A) |  (0 << FOC0B) |	// Forced Output : 
      (0 << WGM02)  |                 //   Waveform Genration Bits 1-0 :
      (0 << CS02)   | (0 << CS01)   |   (0 << CS00);      //   Clock Select bits:
      // 2/1/0
      // 0 0 1       No prescaling   - ok
      // 0 1 0    8 (From prescaler) - ok
      // 0 1 1   64 (From prescaler) - ok
      // 1 0 0  256 (From prescaler) - ok
      // 1 0 1 1024 (From prescaler) - ok
      // 1 1 0 External Clock Source on T0 (Falling Edge)
      // 1 1 1 External Clock Source on T0 (Rising Edge)
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //                 Timer / Counter 1 Setup
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  TCCR1 = // ------------ Timer/Counter1 Control Register ------------
      (0 << CTC1)   |	                // Clear Timer/Counter on Compare Match 
      (1 << PWM1A)  |	                //PWM Enable A for OCR1A: PB1
      (1 << COM1A1) | (0 << COM1A0) | //  Compare Match output Mode OC1A
	  //  COM1A1 / COM1A0 
	  //		0		0		:  Disconnected
	  //		0		1		:  Toggle OC1A  : Outputs on OC1A (PB1) & !OC1A  (PB0)
	  //		1		0		:  Clear OC1A	  : Outputs on OC1A (PB1) Only
	  //		1		1		:  Set OC1A     : Outputs on OC1A (PB1) Only
      (1 << CS13)   | (0 << CS12)   | (1 << CS11)  | (0 << CS10);    //     Clock Select
      // 3/2/1/0
      // 0 0 0 0     T/C1 stopped T/C1 stopped
      // 0 0 0 1     P
      // 0 0 1 0     2
      // 0 0 1 1     4
      // 0 1 0 0     8
      // 0 1 0 1    16
      // 0 1 1 0    32
      // 0 1 1 1    64 
      // 1 0 0 1   256 
      // 1 0 0 1   256 
      // 1 0 1 0   512 
      // 1 0 1 1  1024 
      // 1 1 0 0  2048
      // 1 1 0 1  4096
      // 1 1 1 0  8192
      // 1 1 1 1 16384
  
  GTCCR = // ------------ General Timer/Counter1 Control Register ------------
      (0 << TSM)    |                     // Timer Syncro Mode (used to reset prescaller timer)
      (1 << PWM1B)  |	                    // PWM 1B Enable 
      (1 << COM1B1) | (0 << COM1B0) |   
	  //	COM1B1		COM1B0
	  //		0			0		:	 DISCONNECTED 
	  //		0			1		:	Toggles	OC1B  : OUTPUTS on BOTH OC1B (PB4) and !OC1B (PB3)
	  //		1			0		:	Clear	OC1B    : Outputs on OC1B (PB4) 
	  //		1			1		:	Set		OC1B    : Outputs on OC1B (PB4) 
      (0 << FOC1B)  | (0 << FOC1A)  |	  // Force Output Compare Match 1B & 1A
      (0 << PSR1)   | (0 << PSR0);		  // Prescaler reset Timmer/ Counter 1 & 0
	  //
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void centerPWM()
{
	 OCR1A = 24;	// controll PWM on OC1A (PB1)
		// OCR1A Values:			Full CW		  35		 2.25ms ON pulse	
		//						        Full CCW		11		 .75ms ON pulse
		//						        Center		  24		 1.5ms ON pulse
	 
	 OCR1B = 24;	// Controls PWM on OC1B (PB4)
	 	// OCR1A Values:			Full CW		  35		 2.25ms ON pulse
	 	//						        Full CCW		11		 .75ms ON pulse
	 	//						        Center		  24		 1.5ms ON pulse
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup(){
  DDRB  |= (1 << PB3); // PB3 setup as Output to turn LED on or off Via I2C
  centerPWM();
  initPWM();
  TinyWireS.begin(I2C_SLAVE_ADDR);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {

 if (TinyWireS.available()) {
    
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    u8Command = TinyWireS.receive();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         if (u8Command == 0x00) {}  // Nothing done here, 
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	else if (u8Command == 0x01) {}  // Nothing done here...
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	else if (u8Command == 0x02) {   // Write to PWMA = OCR1A, Servo output on PB1
      u8PwmA = TinyWireS.receive();
	  if (u8PwmA > 35) OCR1A = 35;		//  these are need for Servos  (can remove is you only want PWM)
	  else if (u8PwmA < 11) OCR1A = 11;	//  these are need for Servos  (can remove is you only want PWM)
	  OCR1A = u8PwmA;
      return;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (u8Command == 0x03) {   // Read from PWMA = OCR1A, Servo Value on PWMA (PB1) 
      TinyWireS.send(u8PwmA);
      return;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (u8Command == 0x04) {   // Write to PWMB = OCR1B , Servo output on PB4
      u8PwmB = TinyWireS.receive();
      if (u8PwmB > 35) OCR1B = 35;		//  these are need for Servos  (can remove is you only want PWM)
      else if (u8PwmB < 11) OCR1B = 11;	//  these are need for Servos  (can remove is you only want PWM)
	  OCR1B = u8PwmB;
	  return;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (u8Command == 0x05) {   // Read from PWMB = OCR1B, Servo Value on PWMB (PB4)
      TinyWireS.send(u8PwmB);
      return;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (u8Command == 0x06) {   //  Write BOTH PWMA & PWMB Servos at once to the same value
      u8PwmB = TinyWireS.receive();
      if (u8PwmB > 35) OCR1B = 35;		//  these are need for Servos  (can remove is you only want PWM)
      else if (u8PwmB < 11) OCR1B = 11;	//  these are need for Servos  (can remove is you only want PWM)
      OCR1B = u8PwmB;
	  OCR1A = u8PwmB;
      return;
    }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    else if (u8Command == 0x07) {	// Write PB3 ... Any value > 0 sets PB3 HIGH .  
	  u8PB3 = TinyWireS.receive();
	  if (u8PB3 > 0) PORTB |= (1 << PB3);
	  else PORTB |= ( 0 << PB3);
      
	}
   
	  
 }
    
}
