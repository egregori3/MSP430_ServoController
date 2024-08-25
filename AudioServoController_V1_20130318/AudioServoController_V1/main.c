//---------------------------------------------------------------------------
//
//	  Written by Eric Gregori ( www.buildsmartrobots.com )
//    Copyright (C) 2012  Eric Gregori
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Version 20130309
//
//---------------------------------------------------------------------------
#include "MSP430G2231.h"


//#define TEST_MODE

//-----------------------------------------------------------------------------
// Modes of operation
//-----------------------------------------------------------------------------
#define MODE_ABSOLUTE	1
#define MODE_RELATIVE	2
#define MODE_DIFF		3

//-----------------------------------------------------------------------------
// Noise Filter Constants
//-----------------------------------------------------------------------------
#define MAX_JITTER_LEVEL	5
#define LED_TIMEOUT			15	// 15*20ms = 300ms

//-----------------------------------------------------------------------------
// Measured periods for frequency
//-----------------------------------------------------------------------------
//	Frequency		7.8Mhz		15.25Mhz	15.25Mhz	
//	1000			508			999			1054
//	2000			253			499			526
//	3000			169			332			350
//	4000			126			249			262
//	5000			101			199			209
//	6000			84			165			174
//	7000			72			142			149
//	8000			63			124			131
//	9000			55			110			116
//	10000			50			99			104
//-----------------------------------------------------------------------------
#define PER_1000				1054
#define PER_2000				526
#define PER_3000				350
#define PER_4000				262
#define PER_5000				209
#define PER_6000				174
#define PER_7000				149
#define PER_8000				131
#define PER_9000				116
#define PER_10000				104

//-----------------------------------------------------------------------------
// Servo Constants
//-----------------------------------------------------------------------------
#define SERVO_CENTER	6000
#define SERVO_MIN		2100
#define SERVO_MAX		9500
#define SERVO_RATE		31	
#define SERVO_TIMEOUT	20	

//-----------------------------------------------------------------------------
// global variables
//-----------------------------------------------------------------------------
volatile unsigned short servo1;
volatile unsigned short servo2;
volatile unsigned char  LED1Timeout, LED2Timeout;
volatile unsigned char  Interrupted;
volatile unsigned long  Counter1, Counter2, Counter3, Counter4, Counter5;  // Keep counter in RAM for timing
volatile unsigned char	servo_timeout;		
volatile unsigned char  mode;
						
//-----------------------------------------------------------------------------
// Initialize the Timer for Servo Operation 
//-----------------------------------------------------------------------------
void InitServos( void )
{
	unsigned char i; 
	
  	servo1	 	= 6000;
  	servo2		= 6000; 
  	 		 
	TACCR0		= 6000;			// PWM pulse width
	TACCR1		= 6000;			// PWM pulse width
	 
	TACCTL0 	= 0x0004;		// Mode = 0, OUT = 1
	TACCTL1 	= 0x0004;		// Mode = 0, OUT = 1

	TAR   		= 0x8FFF;  		// Offset Timer1 Overflow interrupt
#if 0	
	// 4 counts / us
	// 8MHz - 0x0262
	//					TASSELx = 10 - SMCLK
	//					IDx		= 01 - /2
	//					MCx		= 10 - Continuous mode: the timer counts up to 0FFFFh.
	//					TACLR   = 0
	//					TAIE	= 1 - Overflow Interrupt Enable
	TACTL 		= 0x0262;
#endif	
	// 4 counts / us
	// 15.25MHz - 0x02A2
	//					TASSELx = 10 - SMCLK
	//					IDx		= 10 - /4
	//					MCx		= 10 - Continuous mode: the timer counts up to 0FFFFh.
	//					TACLR   = 0
	//					TAIE	= 1 - Overflow Interrupt Enable
	TACTL 		= 0x02A2;
}

//-----------------------------------------------------------------------------
// initialize the Clock for 8Mhz Operation - MCLK = SMCLK = DCO
//-----------------------------------------------------------------------------
void InitClock( void )
{
	WDTCTL = WDTPW + WDTHOLD;				// Stop WDT

	// RSELx = 13, DCOx = 3, MODx = 0          7.8  MHz
	// RSELx = 15, DCOx = 3, MODx = 0         15.25 MHz
	BCSCTL1 = 15;        					// RSELx
  	DCOCTL  = (0x03<<5);					// DCOx|MODx
	BCSCTL2 = 0;							// MCLK = SMCLK = DCO
}

//-----------------------------------------------------------------------------
// Initialize GPIO Pins
//
// PIN	GPIO		DIR		FUNCTION
// 		----		---		--------
//  2	P1.0		OUT		LED
//  3	P1.1		TIMER	Timer PWM
//  4	P1.2		OUT		LED
//  5	P1.3		IN		Jumper
//  6	P1.4		OUT		Jumper
//  7	P1.5		IN		Jumper
//  8	P1.6		IN		Audio IN - GPIO Edge Detect
//  9	P1.7		N/A		N/A
//
// 12	P2.6		TIMER	Timer PWM
// 13	P2.7		N/A		N/A
//-----------------------------------------------------------------------------
void InitPins( void )
{
	// PxDIR - 0=in, 1=out
	// PxREN - 0=pullup/down disable
	// PxIE  - 0=interrupt disable
	// PxIES - 0=Low-to-High
	// PxIFG - 0=no interrupt pending
	// PxOUT
	// PxIN
	P1IE  = 0;								// Pin Interrupt Disable
	P1REN = 0;								// Pullup/down disable
	P1SEL = BIT1;							// P1.1=TA0.0 - servo
	P2SEL = BIT6;							// P2.6=TA0.1 - servo

	P1DIR = BIT0|BIT1|BIT2|BIT4;			// P1.6=input, P1.0,1,2,4=out
	P2DIR = BIT6;							// P2.6=out - servo
	
	P1IES = BIT6;							// High to low	
}

//-----------------------------------------------------------------------------
// main()
//
//		
// 3 modes, absolute, relative, diff drive
//
// No Jumpers = diff drive
//
// Jump 1 = absolute
//
// Jump 2 = relative
// 		servo stops after timeout of no signal
//
//-----------------------------------------------------------------------------
void main( void )
{
	unsigned long 	sum;
	unsigned short 	temp_short;
	unsigned short  Last1, Last2;

	InitClock();
	InitPins();	
	InitServos();
	
	P1OUT |= BIT4;							// Set output to one

	Counter1     		= 0;
	Counter2     		= 0;
	Counter3     		= 0;
	Counter4     		= 0;
	Counter5     		= 0;
	Interrupted 		= 0;
	LED1Timeout			= 0;
	LED2Timeout			= 0;
							
	mode = MODE_DIFF;
	if( P1IN & BIT3 ) 
		mode = MODE_ABSOLUTE;
		
	if( P1IN & BIT5 ) 
		mode = MODE_RELATIVE;
	
	__bis_SR_register(GIE);       			// interrupts enabled
			
#ifdef TEST_MODE
	while( 1 )
	{
		P1OUT &= ~BIT0;
		P1OUT &= ~BIT2;
		servo1 = 6000;
		servo2 = 6000;
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		
		P1OUT &= ~BIT0;
		servo1 = 9000;
		servo2 = 9000;
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		
		P1OUT &= ~BIT2;
		servo1 = 3000;
		servo2 = 3000;
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		P1OUT |= (BIT0|BIT2);
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
		for( LED1Timeout=1; LED1Timeout; );
	}
#endif			
								
	while(1)
	{
		Interrupted = 0;

		//
		// Wait for edge
		//
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 );
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 );
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 )
			Counter1++;
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 )
			Counter2++;
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 )
			Counter3++;
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 )
			Counter4++;
		P1IFG = 0;
		while( (P1IFG & BIT6) == 0 )
			Counter5++;

		if( Interrupted ) 
			goto Done;
			
		servo_timeout = 0;
		sum = 0;
		if( abs(Counter1 - Counter2) < MAX_JITTER_LEVEL  )
		{
			if( abs(((Counter1+Counter2)/2)-Counter3) < MAX_JITTER_LEVEL  )
			{
				sum = (Counter1+Counter2+Counter3)/3;
				goto Control;
			}			
		}	
		if( abs(Counter2 - Counter3) < MAX_JITTER_LEVEL  )
		{
			if( abs(((Counter2+Counter3)/2)-Counter4) < MAX_JITTER_LEVEL  )
			{
				sum = (Counter2+Counter3+Counter4)/3;
				goto Control;
			}			
		}		
		if( abs(Counter3 - Counter4) < MAX_JITTER_LEVEL  )
		{
			if( abs(((Counter3+Counter4)/2)-Counter5) < MAX_JITTER_LEVEL  )
			{
				sum = (Counter3+Counter4+Counter5)/3;
				goto Control;
			}			
		}	
		
		goto Done;
		
Control:
#if 1	
//-----------------------------------------------------------------------
// MODE_RELATIVE
//
// 7000 - 8000 = +relative position servo1
// 8000 - 9000 = stop servo1			
// 9000 - 9999 = -relative position servo1	
//
// 3000 - 4000 = +relative position servo2
// 5000 - 6000 = stop servo2
// 6000 - 7000 = -relative position servo2
//
// servo stops after timeout of no signal
//-----------------------------------------------------------------------				
		// Servo = 2000 to 10000  (8000),  6000 center	
		if( mode == MODE_RELATIVE )
		{			
			if( (sum < PER_7000) && (sum > PER_10000) )
			{
				P1OUT &= ~BIT0;
				LED1Timeout = 0;
			}
			else
				P1OUT |= (BIT0);
			
			if( (sum < PER_3000) && (sum > PER_7000) )
			{
				P1OUT &= ~BIT2;
				LED2Timeout = 0;
			}
			else
				P1OUT |= (BIT2);
			
			if( (sum < PER_5000) && (sum > PER_6000) )
				servo2 = 0;

			if( (sum < PER_8000) && (sum > PER_9000) )
				servo1 = 0;
								
			if( (sum < PER_7000) && (sum > PER_8000) )
				servo1 = ((PER_7000-sum)); // 18
			else if( (sum < PER_9000) && (sum > PER_10000) )
				servo1 = 31+((PER_9000-sum)*2); // 22
			else
				servo1 = 0;

			if( (sum < PER_3000) && (sum > PER_4000) )
				servo2 = ((PER_3000-sum)/3); // 27
			else if( (sum < PER_6000) && (sum > PER_7000) )
				servo2 = 31+((PER_6000-sum)); // 23
			else
				servo2 = 0;	
				
			goto Done;
		} // if( mode == MODE_RELATIVE )
#endif
#if 1		
//-----------------------------------------------------------------------
// MODE_DIFF
//
//    8000 to 8999 - forward, forward
//	  6000 to 6999 - backward, backward
//	  7000 to 7999 - forward, backward
//	  9000 to 9999 - backward, forward
//	  quite - stop
//-----------------------------------------------------------------------				
		// Servo = 2000 to 10000  (8000),  6000 center			
		if( mode == MODE_DIFF )
		{
			if( (sum > PER_10000) && (sum < PER_9000) ) // 9500
			{
				// servo = 6000 to 2000
				servo1 = 6000-((PER_9000 - sum)*(4000/(PER_9000-PER_10000)));
				servo2 = servo1;
				P1OUT &= ~BIT0;
				LED1Timeout = 0;
			}
			else if( (sum > PER_9000) && (sum < PER_8000) ) // 8500
			{
				// servo = 6000 to 2000 
				servo1 = 6000-((PER_8000 - sum)*(4000/(PER_8000-PER_9000)));
				// servo = 6000 to 10000
				servo2 = 6000+((PER_8000 - sum)*(4000/(PER_8000-PER_9000)));
				P1OUT &= ~BIT0;
				LED1Timeout = 0;
				P1OUT &= ~BIT2;
				LED2Timeout = 0;
			} 
			else if( (sum > PER_8000) && (sum < PER_7000) ) // 7500
			{
				// servo = 6000 to 10000
				servo1 = 6000+((PER_7000 - sum)*(4000/(PER_7000-PER_8000)));
				servo2 = servo1;
				P1OUT &= ~BIT2;
				LED2Timeout = 0;
			} 
			else if( (sum > PER_7000) && (sum < PER_6000) ) // 6500
			{
				// servo = 6000 to 2000 
				servo2 = 6000-((PER_6000 - sum)*(4000/(PER_6000-PER_7000)));
				// servo = 6000 to 10000
				servo1 = 6000+((PER_6000 - sum)*(4000/(PER_6000-PER_7000)));
				P1OUT &= ~BIT0;
				LED1Timeout = 0;
				P1OUT &= ~BIT2;
				LED2Timeout = 0;
			} 	
		}
#endif
#if 1		
//-----------------------------------------------------------------------
// MODE_ABSOLUTE
//
//	Servo1				Servo2
// 	Freq = servo
//	------------		----------
//	1000 = 2064			1540 = 2032
//	1010 = 2352			1550 = 2160
//	1100 = 4944			1575 = 2512
//	1200 = 7376			1600 = 2800
//	1300 = 9424			2000 = 6800
//	1310 = 9616			2500 = 9968
//	1320 = 9808
//	1330 = 10000
//-----------------------------------------------------------------------				
		// Servo = 2000 to 10000  (8000),  6000 center	
		if( mode == MODE_ABSOLUTE )
		{
			if( (sum > (PER_1000-250)) && (sum < PER_1000) )
			{	
				sum = 2000+((PER_1000-sum)*(8000/250));
				if( sum>Last1 )
					temp_short = sum-Last1;
				else
					temp_short = Last1-sum;
						
				if( temp_short > 100 )
				{	
					servo1 = sum;
					Last1 = sum;
				}
				P1OUT &= ~BIT0;
				LED1Timeout = 0;
			}
			else if( (sum > (PER_1000-600)) && (sum < (PER_1000-350)) )
			{		
				sum = 2000+(((PER_1000-350)-sum)*(8000/250));
				if( sum>Last2 )
					temp_short = sum-Last2;
				else
					temp_short = Last2-sum;
						
				if( temp_short > 100 )
				{	
					servo2 = sum;
					Last2 = sum;
				}
				P1OUT &= ~BIT2;
				LED2Timeout = 0;
			}
		} // mode ABSOLUTE
#endif
	
Done:		
		Counter1     		= 0;
		Counter2     		= 0;
		Counter3     		= 0;
		Counter4     		= 0;
		Counter5     		= 0;
	} // while(1)						
}


//-----------------------------------------------------------------------------
// Interrupt Service Routine (ISR) - Timer Rollover
// 
// Every timer rollover, set PWM LOW for servo[] counts.
//-----------------------------------------------------------------------------
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer1_A1 (void)
{
	volatile unsigned short i;
	
	Interrupted = 1;
	
	if( TAIV == TAIV_TAIFG )
	{		
		// Set to mode 0
		TACCTL0 	= 0; 			//0x0004;
		TACCTL1 	= 0; 			//0x0004;
		i = TAR;
		while( TAR == i );
		
		if( servo_timeout < SERVO_TIMEOUT )
			++servo_timeout;
		else if( mode == MODE_DIFF )
		{
			servo1 = 6000;
			servo2 = servo1;
		}
				
		// TACCR0, servo[0]
		if( servo1 > 2000 )
			TACCR0		= servo1; // 2000 - 10000
		else
		{
			if( servo_timeout < SERVO_TIMEOUT )
			{
				if( servo1 <= SERVO_RATE )
				{
					if( TACCR0 < SERVO_MAX )
						TACCR0 += servo1;		// 1-31 incs
				}
				else
				{
					if( TACCR0 > SERVO_MIN )
						TACCR0 -= (servo1-SERVO_RATE);	// 32-63 decs
				}
			}				
		}

		// TACCR1, servo[1]
		if( servo2 > 2000 )
			TACCR1		= servo2; // 2000 - 10000
		else
		{
			if( servo_timeout < SERVO_TIMEOUT )
			{
				if( servo2 <= SERVO_RATE )
				{
					if( TACCR1 < SERVO_MAX )
						TACCR1 += servo2;		// 1-31 incs
				}
				else
				{
					if( TACCR1 > SERVO_MIN )
						TACCR1 -= (servo2-SERVO_RATE);	// 31-63 decs
				}
			}
		}

		TACCTL0 	= 0x0020; 		// 0x00A4;
		TACCTL1 	= 0x0020; 		// 0x00A4;

		// Turn off LED's after LED_TIMEOUT * 20ms
		if( ++LED1Timeout > LED_TIMEOUT )		
		{
			P1OUT |= (BIT0);
			LED1Timeout = 0;
		}		
		if( ++LED2Timeout > LED_TIMEOUT )		
		{
			P1OUT |= (BIT2);
			LED2Timeout = 0;
		}	
  	}		
}



