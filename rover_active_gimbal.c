#include <stdlib.h>
#include <stdio.h>
#include <p89v51rd2.h>

#include "angle_menu.h"
#include "y_axis.h"
#include "z_axis.h"
#include "numbers.h"
#include "neg_sign.h"
#include "pos_sign.h"

#include "input_angles.h"
#include "invalid_angles.h"
#include "nunchuck_menu.h"
#include "joystick.h"
#include "math.h"



/****************************************************************/
// PORT DEFINITIONS
#define period_x_pin P0_0     // Freq from cap 1
#define period_y_pin P0_1	  // Freq from cap 2
	
#define timer_x_enable P0_3	  // 555 enable for cap 1
#define timer_y_enable P0_4   // 555 enable for cap 2
	
#define x_pin0	P0_2		  // P for cap 1 motor 
#define x_pin1	P2_7		  // G for cap 1 motor

#define y_pin0	P2_5          // P for cap 2 motor
#define y_pin1	P2_6          // G for cap 2 motor

#define D_I  P3_6			  // For LCD
#define R_W  P3_5
#define E1   P3_3
#define E2   P3_4
#define RST  P3_2

#define SPI_EN P1_4			  // SPI chip enable

#define col0 P1_1 			  // Keypad
#define col1 P0_5 
#define col2 P1_0 
#define row0 P1_3 
#define row1 P0_7  
#define row2 P1_2 
#define row3 P0_6 

#define SDA P2_4			 // Nunchuck Data	
#define SCL P2_3			 // Nunchuck Clock

/****************************************************************/
// OTHER MACROS
#define CLK 22118400L	// P89V51RD2 Clock Speed

#define BAUD 9600L		// For serial comm
#define TIMER_2_RELOAD (0x10000L-(CLK/(32L*BAUD)))	

#define SPEEDFACTOR	200	// For PI control mechanism for motors
#define MAXSPEED 90
#define MINSPEED 15
#define INITSPEED 90 
#define TOLERANCE 10

#define JOYSTICKPWM 70	// When in joystick mode, motors move with this const pwm

#define I2C_DELAY 0x0F	// For delay I2C bus
 
#define FREQ 10000L		//We want timer 0 to interrupt every 100 microseconds ((1/10000Hz)=100 us)
#define TIMER0_RELOAD_VALUE (65536L-((CLK)/(12*FREQ)))

//Coefficients for Period(angle)
#define a3 0.0
#define a2 0.0	//
#define a1 (420.0/180.0)	//slope 
#define a0 544.0			//0degrees period

//Coefficients for Angle(Period)
#define b3 0.0
#define b2 0.0	//
#define b1 (180.0/420.0)	//slope 
#define b0 544.0			//0degrees period

/****************************************************************/
// GLOBAL VARIABLES
volatile unsigned char pwmcount;
volatile bit pwmcountstat = 0;
volatile unsigned char pwm_x = 0; 	// PWM for cap 1 motor	
volatile unsigned char pwm_y = 0;	// PWM for cap 2 motor
	
volatile float current_period_x = 0;	// Current period of cap 1 in us
volatile float current_period_y = 0;	// Current period of cap 2 in us

volatile float error_x = 0;	// Diff b/w period to be set and current period
volatile float error_y = 0;	// Diff b/w period to be set and current period

int set_period_x = 700;		// Period to set for cap 1
int set_period_y = 780;		// Period to be set for cap 2

volatile unsigned char temp;

unsigned char arraycount = 0;
bit num1or2 = 0;
pdata char y_number[] = {0, 0, 0}; 	// Angle digits set by user for cap 1
pdata char z_number[] = {0, 0, 0};		// Angle digits set by user for cap 2
bit sign_y = 0;	
bit sign_z = 0;
pdata float realAngle1 = 0;	// Angle for cap 1
pdata float realAngle2 = 0;	// Angle for cap 2

unsigned int joystick_count = 0;	// For joystick menu

int state = 1;

unsigned char values[6];	// Array storing the values read from the nunchuck
volatile bit m1;	// Direction for cap 1 motor
volatile bit m2;	// Direction for cap 2 motor

/***************************************************************/
// FUNCTION DECLARATIONS
void InitTimer0(void);
float angleFit(float);
float period_Fit(float);
void angleDigits(float,float, unsigned char*, unsigned char*);
void printInput();
void Wait1s(void);	
void delay(unsigned int n);
void init();
void Comleft(char i);
void Comright(char i);
void Writeright(char i);
void Writeleft(char i);
void Graphic(unsigned char *img);
void bothSides(char i);
void print_z_axis(unsigned char *img, int x);
void check_sign_y(bit sign_y, int);
void check_sign_z(bit sign_z, int);
void state_two(void);
void state_four(void);
unsigned char sendToLCD(unsigned char) critical;
bit is_star_pressed();
bit is_zero_pressed();
bit is_pound_pressed();
void state_six();

void wait();	// For nunchuck
void nunchuck_init() critical;
void conv() critical;
void read() critical;
void I2C_delay();	// I2C Functions
void I2C_clock();
void I2C_start();
void I2C_stop();
bit I2C_write(unsigned char dat);
unsigned char I2C_read(void);
void I2C_ack();
void I2C_noack();
void motor_nck();	// Motor function for joystick control


/***************************************************************/
// INTERRUPTS
void pwmcounter (void) interrupt 1
{
	TR0 = 0;
	TH0 = TIMER0_RELOAD_VALUE/0x100;
	TL0 = TIMER0_RELOAD_VALUE%0x100;
	
	if(++pwmcount>99) { pwmcount=0; pwmcountstat = !pwmcountstat; }

	if (state != 6) {
		// Cap 1 Period Measurement
		if (pwmcount == 0 && pwmcountstat == 0) {
			timer_y_enable = 0;
			timer_x_enable = 1;
			TR1 = 0;
			TH1 = 0;
			TL1 = 0;
			if (period_x_pin) 
			{
				while (period_x_pin);
				TR1 = 1;
				while (!period_x_pin);
				TR1 = 0;
			}
			else
			{	
				while (!period_x_pin);
				TR1 = 1;
				while (period_x_pin);
				TR1 = 0;
			}
		
			current_period_x = (float) (1.08 * (TH1 * 256 + TL1));
			timer_x_enable = 0;
			
			// Calculate motor speed for Cap 1
			error_x = (float) (set_period_x - current_period_x);
			temp =  (unsigned char) (INITSPEED * (abs(error_x) / (float) SPEEDFACTOR));
			pwm_x = temp;
					
			
			if(pwm_x > MAXSPEED)
			{
				pwm_x = MAXSPEED;
			}
			else if (pwm_x < MINSPEED)
			{
				pwm_x = 0;
			}	
		
		}
		
		// Cap 2 measurement
		if (pwmcount == 0 && pwmcountstat == 1) {
			timer_x_enable = 0;
			timer_y_enable = 1;
			TR1 = 0;
			TH1 = 0;
			TL1 = 0;
		if (period_y_pin) 
		{
			while (period_y_pin);
			TR1 = 1;
			while (!period_y_pin);
			TR1 = 0;
		}
		else
		{	
			while (!period_y_pin);
			TR1 = 1;
			while (period_y_pin);
			TR1 = 0;
		}
		
		current_period_y = (float) (1.08 * (TH1 * 256 + TL1));
		timer_y_enable = 0;
		
		// Calculate motor speed for Cap 2
		error_y = (float) (set_period_y - current_period_y);
		temp =  (unsigned char) (INITSPEED * (abs(error_y) / (float) SPEEDFACTOR));
		pwm_y = temp;
				
		
		if(pwm_y > MAXSPEED)
		{
			pwm_y = MAXSPEED;
		}
		else if (pwm_y < MINSPEED)
		{
			pwm_y = 0;
		}	
		
	}
	
	
	// Send signal to motors
	if (current_period_x < (set_period_x - TOLERANCE))
	{
		x_pin0 = (pwm_x > pwmcount)?1:0;
		x_pin1 = 0;
		
	}
	else
	if (current_period_x > (set_period_x + TOLERANCE))
	{
		x_pin0 = 0;
		x_pin1 = (pwm_x > pwmcount)?1:0;
	}
	else
	{	
		x_pin0 = x_pin1 = 0;
	}
	
	if (current_period_y < (set_period_y - TOLERANCE))
	{
		y_pin0 = (pwm_y > pwmcount)?1:0;
		y_pin1 = 0;
	}
	else
	if (current_period_y > (set_period_y + TOLERANCE))
	{
		y_pin0 = 0;
		y_pin1 = (pwm_y > pwmcount)?1:0;
	}
	else
	{	
		y_pin0 = y_pin1 = 0;
	}
	
	} 
	else 
	{	
		/**************************************************
		// Joystick Motor control */
		
		if(m1 == 1)
			x_pin0 = (pwm_x>pwmcount)?1:0;
		else 
			x_pin1 = (pwm_x>pwmcount)?1:0;
	
		if(m2 == 1)
			y_pin0 = (pwm_y>pwmcount)?1:0;
		else 
			y_pin1 = (pwm_y>pwmcount)?1:0;
	}
		
	
	TR0 = 1;
}	
	

/**************************************************************/
// MAIN
void main (void)
{
	unsigned char i;
	
    P0 = 0xC3;
    P1 = 0x4C;
    P2 = P2 & 0x07;
    P3 = P3 & 0x83; 
	
	SPI_EN = 1;
	init();
    
    setbaud_timer2(TIMER_2_RELOAD); // Initialize serial port (for Hyperterminal) using timer 2
	
    
    timer_y_enable = 0;
	timer_x_enable = 0;
	
	y_pin0 = y_pin1 = 0;
      
    InitTimer0();
    
    nunchuck_init();
    wait();
    
    state = 1;
	
	while(1) 
	{
		if (state == 2) //Measurement Display -> Input Menu
		{
			if (is_star_pressed() == 0) 
			{ 
				while(!is_star_pressed()){} //Debounce KEY_3
				state = 3; 
			}
		}
		
	
		if (state == 2) //Measurement Display -> Nunchuck Operation
		{
			if (is_pound_pressed() == 0) 
			{
				while(!is_pound_pressed()){}	//Debounce KEY_3
				state = 5;
			}
	
		}
		
		if (state == 2) //Measurement Display -> Input Menu
		{
			if (is_zero_pressed() == 0) 
			{
				while(!is_zero_pressed()){}	//Debounce KEY_3
				state = 3;
			}	
		}
		
		if (state == 6) //Nunchuck Operation -> Input Menu
		{
			if (is_pound_pressed() == 0) 
			{
				while(!is_pound_pressed()){}	//Debounce KEY_3
				state = 1;
			}
		}
	
		
		switch (state)
		{
		
			case 1: 
					Graphic(angle_menu);
					state = 2;
					break;
			
			case 2:
					//Measurement Display	
					state_two();
					break;
			
			case 3: 
					Graphic(input_angles);
					for(i=0;i<3;i++)
						{y_number[i]=0; z_number[i]=0;}
					state = 4;
					break;
			
			case 4:
					state_four();
					//Input Angle Menu
					break;
			
			case 5:
					Graphic(nunchuck_menu);
					state = 6;
					break;
			
			case 6:
					state_six();
					//Operate using Nunchuck
					break;
			
			default:
					break;
		}


	}
	
	
}


/***************************************************************/
// FUNCTION DEFINITIONS
void InitTimer0 (void)
{
	TR0 = 0; 
	TR1 = 0;
	TMOD = (TMOD&0xf0)|0x01; 
	TMOD = (TMOD&0x0f)|0x10;
	TH0 = TIMER0_RELOAD_VALUE/0x100;
	TL0 = TIMER0_RELOAD_VALUE%0x100;
	
	EA=1;  // Enable global interrupts
	ET0=1; // Enable timer 0 interrupt for pwm motor
	
	TR0 = 1; 
}


void angleDigits(float rA, float rB, unsigned char *y_num, unsigned char *z_num)
// To Populate the array angleGLCD[] (to be displayed on GLCD)
{

	int y, yRemain, z, zRemain;
	char j;
	y = yRemain = z = zRemain = j = 0;

	//Set flag neg_sign according to the sign of theta
	// 0 = positive; 1 = negative
	sign_y=(rA<0)?1:0;	
	sign_z=(rB<0)?1:0;	
		
	// To Populate the array angleGLCD[] (to be displayed on GLCD)
	rA *= 10.0;
	if(sign_y)
		rA=-rA;
			
	rB *= 10.0;
	if(sign_z)
		rB=-rB;
			
	y = (int) rA;
	z = (int) rB;
		
	for(j=2; j>=0; j--)
	{
		yRemain = y % 10;
		zRemain = z % 10;
			
		y/=10;
		z/=10;
			
		y_num[j] = (yRemain>0) ? yRemain:0;
		z_num[j] = (zRemain>0) ? zRemain:0;	
	}
		
}

float period_Fit(float angle)
{

	/******************************************
		Camera angle = -Capacitance Angle;
	*******************************************
	Linear fit:		Period(angle) = a1x-a0	
	Cubic Spline:	Peroid(anlge) = a3x^3+a2x^2+a1x-a0 */

	float x = angle;
			
	//Bound theta between [-90, 90]	
	if(x < -90)		
		x = 90;
	else if (x > 90)
		x = 90;		

	return (a3*x*x*x+a2*x*x+a1*x+a0);
}


float angleFit(float period)
{

	/******************************************
	Camera angle = -Capacitance Angle;
	*******************************************	
	Linear fit:		Angle(period) = c1x-c0	
	Cubic Spline:	Angle(period) = c3x^3-c2x^2+c1x-c0 */

	float x = period, theta=0;

	//Bound period between ~~[544, 1000]							
	if(period < 544)			
		period = 544;
	else if (period > 1000)
		period = 1000;

	
	theta = (b1 * (x - b0)) - 90.0;
					
	//Bound theta between [-90, 90]							
	if(theta < -90)			
		theta = -90;
	else if (theta > 90)
		theta = 90;
	
	return theta;
}

void delay(unsigned int n)
{
	unsigned int i,j;
	for (i=0;i<n;i++)
  	{
  		for (j=0;j<350;j++)
  			{;}
  	}
  			
}

void Comleft(char i)
{
	sendToLCD(i);
	R_W = 0;
	D_I = 0;
	E1 = 1;
//	delay(2);
	E1 = 0;
}

void Comright(char i)
{
	sendToLCD(i);
	R_W = 0;
	D_I = 0;
	E2 = 1;
//	delay(2);
	E2 = 0;
}

void Writeleft(char i)
{
	sendToLCD(i);
	R_W = 0;
	D_I = 1;
	E1 = 1;
//	delay(2);
	E1 = 0;
}

void Writeright(char i)
{
	sendToLCD(i);
	R_W = 0;
	D_I = 1;
	E2 = 1;
//	delay(2);
	E2= 0;
}

void bothSides(char i)
{
	Comleft(i);
	Comright(i);
}


void init(void)
{
	
	sendToLCD(0x00);
	P3 = P3&0x83;
	
	RST = 0;	//	Reset RST
	delay(1);
	RST = 1;	//	Reset RST= M68 Interface
	delay(10);
	D_I = 0;
	E1 = 1;
	E2 = 1;
	R_W = 1;
	
	bothSides(0xE2);
	delay(10);
	bothSides(0xA4);
	bothSides(0xA9);
	bothSides(0xA0);
	bothSides(0xEE);
	bothSides(0xC0);
	bothSides(0xAF);
	
}

void print_y_axis(unsigned char *img, int x){

	unsigned char m, n;
	char page = 0xB9;
	x = x * 72;
	img = img+x;
	for(m=0;m<3;m++)
	{
		bothSides(page);
		bothSides(0x03);
		for(n=0;n<24;n++)
		{
			Writeleft(*img);
			img++;
		}
		page++;
	}
}

void print_z_axis(unsigned char *img, int x){

	unsigned char m, n;
	char page = 0xB9;
	x = x * 72;
	img = img+x;
	for(m=0;m<3;m++)
	{
		bothSides(page);
		bothSides(0x03);
		for(n=0;n<24;n++)
		{
			Writeright(*img);
			img++;
		}
		page++;
	}
}

void y_angle(unsigned char *img, int x, int y){

	unsigned char n;
	bothSides(0xBA);
	bothSides(y);
	x = x*5;
	img = img+x;
	for(n=0;n<5;n++)
	{
		Writeleft(*img);
		img++;
		y++;
		bothSides(y);	
	}
}

void z_angle(unsigned char *img, int x, int y){

	unsigned char n;
	bothSides(0xBA);
	bothSides(y);
	x = x*5;
	img = img+x;
	for(n=0;n<5;n++)
	{
		Writeright(*img);
		img++;
		y++;
		bothSides(y);	
	}
}

void check_sign_y(bit sign_y, int x) 
{

	if (sign_y) { y_angle(neg_sign, 0, x); }
	else { y_angle(pos_sign,0, x);}

}

void check_sign_z(bit sign_z, int x) 
{

	if (sign_z) { z_angle(neg_sign, 0, x); }
	else {z_angle(pos_sign,0, x); }

}


void Graphic(unsigned char *img)
{
	unsigned char m, n;
	char page = 0xB8;
	for(m=0;m<4;m++)
	{
		bothSides(page);
		bothSides(0x00);
		for(n=0;n<61;n++)
		{
			Writeleft(*img);
			img++;
		}
		for(n=0;n<61;n++)
		{
			Writeright(*img);
			img++;
		}
		page++;
	}
}

void compare_angles(void) {

	int angle_pic_y = (int)ceilf(realAngle1/7.5) + 12;
	int angle_pic_z = (int)ceilf(realAngle2/7.5) + 12;
		
	print_y_axis(y_axis, angle_pic_y);
	print_z_axis(z_axis, angle_pic_z);
	
}


void state_two(void){

	unsigned char i, j;
	int column = 35;

/*		
	GetCount1(); //Gets the period from Timer 0
	period = (1.08*(TH0*256+TL0));  // convert period of Timer0
*/
	realAngle1 = angleFit(current_period_x);
	
/*
	GetCount2(); //Gets the period from Timer 0
	period = (1.08*(TH0*256+TL0));  // convert period of Timer0
*/

	realAngle2 = angleFit(current_period_y);

	//Make Use of realAngle2
	
	angleDigits(realAngle1,realAngle2, y_number, z_number);

	compare_angles();

	for(i = 0; i<3; i++) 
	{
		check_sign_y(sign_y, 29);			
		y_angle(numbers, y_number[i], column); // Digit 1

		if (column != 41) { column = column + 6; }
		else { 
		column = column + 9;
		}
	}
		
	column = 35;
	
	for(j = 0; j<3; j++) 
	{
		check_sign_z(sign_z, 29);
		z_angle(numbers, z_number[j], column); // Digit 1
		if (column != 41) { column = column + 6; }
		else { 
			column = column + 9;
		}
	}

	i = 0;
	j = 0;
	column = 35;
		
	// delay(1000);
}


void printInput()
{

	int i = 0;
	int j = 0;
	int l = 0;

	int column2=21;

	for(i = 0; i<3; i++) 
	{
		
		check_sign_y(sign_y, 15);			
		y_angle(numbers, y_number[i], column2); // Digit 1

		if (column2 != 27) { column2 = column2 + 6; }
		else { 
			column2 = column2 + 9;
		}
	}
		
	column2 = 23;
		
	for(j = 0; j<3; j++) 
	{
		check_sign_z(sign_z, 17);
		z_angle(numbers, z_number[j], column2); // Digit 1
		if (column2 != 29) { column2 = column2 + 6; }
		else { 
			column2 = column2 + 9;
		}
	}

	i = 0;
	j = 0;
	column2 = 21;

}


bit is_zero_pressed()
{
 	col0 = 1; col1 = 0; col2 = 1;
	if (row3 == 0) return 0;//false;
	return 1; //true;
		
}

bit is_star_pressed()
{
	col0 = 0; col1 = 1; col2 = 1;
	if (row3 == 0) return 0;//false;
	return 1;//true;
}

bit is_pound_pressed()
{
	col0 = 1; col1 = 1; col2 = 0;
	if (row3 == 0) return 0;//false;
	return 1;//true;
}

unsigned char sendSPI(unsigned char SPI_Temp) critical
{
	unsigned char i;
	SPDAT = SPI_Temp;  //transmit SPI Data
	//while ((SPCFG & 0x80)==0);
	//while (SPIF==0);
	for (i=0; i<255; i++);
	SPI_Temp = SPDAT;
	
	return SPI_Temp;
}

unsigned char sendToLCD(unsigned char byte_sent) critical
{
	unsigned char byte_received;
	
	SPCTL = 0x53;
	SPI_EN = 0;
	byte_received = sendSPI(byte_sent);
	SPI_EN = 1;
	SPCTL = SPCTL ^ 0x40;
	
	return byte_received;	
}
	

void state_four(void) 
//Input Angles
{
	unsigned char keypressed = 0;
	col0 = col1 = col2 = 0;
	if (row0 != 1 || row1 != 1 || row2 != 1 || row3 != 1)
	{	
		// Start getting one key
		col0 = 0; col1 = 1; col2 = 1;
		if (row0 == 0)
		{	keypressed = 1; }
		else if (row1 == 0)
		{	keypressed = 4;}
		else if (row2 == 0)
		{	keypressed = 7;}
		else if (row3 == 0)
			{
				if (num1or2==0) {sign_y=!sign_y;}
			 	else {sign_z=!sign_z;}
			 	while (row3==0);
			 	return;
			}
		else	
		{	col0 = 1; col1 = 0; col2 = 1;
			if (row0 == 0)
			{	keypressed = 2;}
			else if (row1 == 0)
			{	keypressed = 5;}
			else if (row2 == 0)
			{	keypressed = 8;}
			else if (row3 == 0)
			{	keypressed = 0;}
				
		else
			{
				col0 = 1; col1 = 1; col2 = 0;
				if (row0 == 0)
				{	keypressed = 3;}
				else if (row1 == 0) 
				{	keypressed = 6;}
				else if (row2 == 0)
				{	keypressed = 9;}
				else if (row3 == 0)
					{
						if (num1or2 == 1) state =1;
						num1or2=!num1or2;
						arraycount=0;
						while(row3==0);
						return;
					}
			}
		}
		while (row0==0 || row1==0 || row2==0 || row3==0);
		if (arraycount==3) arraycount=0;
		if (num1or2 == 0) y_number[arraycount] = keypressed;
		else z_number[arraycount] = keypressed;
		arraycount++;
	}

	printInput();
	return;
}

void print_joystick(unsigned char *img, int x) {

	unsigned char n, m;
	char page = 0xB8;
	x = x*16;
	img = img+x;
	for(m=0;m<2;m++)
	{
		bothSides(page);
		bothSides(20);
		for(n=0;n<8;n++)
		{
			Writeleft(*img);
			img++;
		}
		page++;
	}
	
}


void state_six(void) 
{
	unsigned char i;
		
	read();
	wait();
	
	
	if(joystick_count == 0) { print_joystick(joystick, 0);}
	else if(joystick_count == 8000) { print_joystick(joystick, 1);}
	else if (joystick_count == 16000) {print_joystick(joystick, 2);}
	else if (joystick_count == 24000) {print_joystick(joystick, 3);}
	else if (joystick_count == 32000) {print_joystick(joystick, 4);}

	if(joystick_count == 40000) {joystick_count = 0;}
	else {joystick_count++;}
	
	for (i=0; i<6; i++)
		{
			values[i] = values[i] ^ 0x17;
			values[i] = values[i] + 0x17;
		}
	/*	DEBUG info
	printf ("\n\nSx = %d, Sy = %d", values[0], values[1]);
	printf("\nAx = %d, Ay = %d, Az = %d", values[2], values[3], values[4]);
	*/
	
	motor_nck();	
	
	
}

void motor_nck()
// bit set for positive direction and cleared for negative
{	

	// For cap 1 motor
	if (values[0] > 160)
	{
		m1 = 1;
		pwm_x = JOYSTICKPWM;

	}
	else
	if (values[0] < 90)
	{
		m1 = 0;
		pwm_x = JOYSTICKPWM;

	}
	else
	{
		pwm_x = 0;
	}
	
	// For cap 2 motor
	if (values[1] > 160)
	{
		m2 = 1;
		pwm_y = JOYSTICKPWM;

	}
	else
	if (values[1] < 90)
	{
		m2 = 0;
		pwm_y = JOYSTICKPWM;

	}
	else
	{
		pwm_y = 0;
	}
	
}



void wait()
{
	_asm;
	push AR1
	push AR0
	push AR2
	
	mov R2, #1
	X3: mov R1, #10
	X2: mov R0, #125
	X1: djnz R0, X1 ; 3 machine cycles-> 3*90ns*149=40us
	djnz R1, X2
	djnz R2, X3	
	
	pop AR2
	pop AR0
	pop AR1
	_endasm;
	
}


void nunchuck_init() critical
{
	I2C_start();
	
	I2C_write(0xA4);
	I2C_write(0x40);
	I2C_write(0x00);
	
	I2C_stop();
}

void conv() critical
{
	I2C_start();
	
	I2C_write(0xA4);
	I2C_write(0x00);
	
	I2C_stop();
}

void read() critical
{
	int i;
	conv();
	wait();
	I2C_start();
	
	I2C_write(0xA5);
	
	for (i=0; i<6; i++)
	{
		values[i] = I2C_read();
		if (i==5)
			I2C_noack();
		else 
			I2C_ack();
	}
	
	I2C_stop();
}

/************************************************
 * I2C Functions
 *
 *
 *
 *************************************************/
void setAsInput()
{
	P2 = P2|0x10;
}

void setAsOutput()
{
	P2 = P2 & 0xe7;
}

void I2C_delay(void)
{
	unsigned char i;

	for(i=0; i<I2C_DELAY; i++);
}

void I2C_clock(void)
{
	I2C_delay();

	SCL = 1;		/* Start clock */

	I2C_delay();    

	SCL = 0;		/* Clear SCL */
}

void I2C_start(void)
{
	setAsOutput(); //SDA & SCL outputs
	if(SCL)
	SCL = 0;		/* Clear SCL */

	SDA = 1;        /* Set SDA */
	SCL = 1;		/* Set SCL */

	I2C_delay(); 

	SDA = 0;        /* Clear SDA */

	I2C_delay(); 

	SCL = 0;        /* Clear SCL */
}

void I2C_stop(void)
{
	setAsOutput(); //SDA & SCL outputs
	if(SCL)	
	SCL = 0;			/* Clear SCL */

	SDA = 0;			/* Clear SDA */
	I2C_delay();

	SCL = 1;			/* Set SCL */

	I2C_delay();

	SDA = 1;			/* Set SDA */
}

bit I2C_write(unsigned char dat)
{
	
	bit data_bit;		
	unsigned char i;	
	setAsOutput();
	for(i=0;i<8;i++)				/* For loop 8 time(send data 1 byte) */
	{
		data_bit = dat & 0x80;		/* Filter MSB bit keep to data_bit */
		SDA = data_bit;				/* Send data_bit to SDA */

		I2C_clock();      			/* Call for send data to i2c bus */

		dat = dat<<1;  
	}

	SDA = 1;			/* Set SDA */

	I2C_delay();	
	
	SCL = 1;			/* Set SCL */
	
	I2C_delay();	

	data_bit = SDA;   	/* Check acknowledge */
	SCL = 0;			/* Clear SCL */

	I2C_delay();

	return data_bit;	/* If send_bit = 0 i2c is valid */		 	
}

unsigned char I2C_read(void)
{
	
	bit rd_bit;	
	unsigned char i, dat;
	setAsInput();
	dat = 0x00;	

	for(i=0;i<8;i++)		/* For loop read data 1 byte */
	{
		I2C_delay();

		SCL = 1;			/* Set SCL */

		I2C_delay(); 

		rd_bit = SDA;		/* Keep for check acknowledge	*/
		dat = dat<<1;		
		dat = dat | rd_bit;	/* Keep bit data in dat */

		SCL = 0;			/* Clear SCL */
	}

	return dat;
}

void I2C_ack()
{
	setAsOutput();
	SDA = 0;		/* Clear SDA */

	I2C_delay();    

	I2C_clock();	/* Call for send data to i2c bus */

	SDA = 1;		/* Set SDA */
}

void I2C_noack()
{
	setAsOutput();
	SDA = 1;		/* Set SDA */

	I2C_delay();

	I2C_clock();	/* Call for send data to i2c bus */

	SCL = 1;		/* Set SCL */
}




	

