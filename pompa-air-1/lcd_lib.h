// Title		: LCDinterface
// author?
// mod kamal

#include <avr/io.h>
#include <util/delay.h>

// used pins on port A
#define LCD_DB4 0    // PORTA.0
#define LCD_DB5 1    // PORTA.1
#define LCD_DB6 2    // PORTA.2
#define LCD_DB7 3    // PORTA.3
#define LCD_E  5     // PORTA.5 Enable
#define LCD_RS 4     // PORTA.4 Register Select


//LCD commands
#define LCD_CLR 		0x01    // clear display
#define LCD_HOME 		0x02    // return home

#define LCD_INC 		0x06    // Increment, display freeze
#define LCD_MOV 		0x10    // Cursor move, not shift

#define LCD_OFF         0x08    // lcd off
#define LCD_ON          0x0C    // lcd on
#define LCD_BLINK_ON	0x0D    // blink on
#define LCD_CURSOR_ON	0x0E    // cursor on
#define LCD_ALL_ON	    0x0F    // cursor on /  blink on
#define LCD_LINE1  		0x80    // cursor Pos on line 1 (or with column)
#define LCD_LINE2  		0xC0    // cursor Pos on line 2 (or with column)

unsigned char chr,data,pos;

// writes a char to the LCD
void LCD_char(unsigned char data)

{
	PORTA = (data&0b11110000) >> 4; //high nibble
	PORTA |= 1<<LCD_RS;
	PORTA |= 1<<LCD_E;
	_delay_ms(2);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(2);

	PORTA = (data&0b00001111); //low nibble
	PORTA |= 1<<LCD_RS;
	PORTA |= 1<<LCD_E;
	_delay_ms(2);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(2);
}


// writes a instruction to the LCD
void LCD_inst(unsigned char inst)
{
	PORTA = (inst&0b11110000) >> 4; //send high nibble
	PORTA &= ~(1<<LCD_RS); // set RS to instructions
	PORTA |= 1<<LCD_E;
	_delay_ms(2);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(2);

	PORTA = (inst&0b00001111); //send low nibble
	PORTA |= 1<<LCD_E;
	_delay_ms(2);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(2);
}

// clear display
void LCDclr(void)
{
	LCD_inst (LCD_CLR);
}

// return home
void LCDhome(void)
{
	LCD_inst (LCD_HOME);
}

// LCD off
void LCDoff(void)
{
	LCD_inst (LCD_OFF);
}

// LCD on
void LCDon(void)
{
	LCD_inst (LCD_ON);
}

// cursor on
void LCDcursor(void)
{
	LCD_inst (LCD_CURSOR_ON);
}

// blink on
void LCDblink(void)
{
	LCD_inst (LCD_BLINK_ON);
}

// cursor all on
void LCDall(void)
{
	LCD_inst (LCD_ALL_ON);
}

//go to first line
void LCDline1 (void)

{
	LCD_inst (0b10000000);

}

//go to second line
void LCDline2 (void)

{
	LCD_inst (0b11000000);

}


// goto position x,y
void LCDgoto (char x,char y)
{

	if (y == 0)
	{
		pos = 0b00000000 + x;
	}

	else if (y == 1)
	{
		pos = 0b01000000 + x;
	}

	LCD_inst (0b10000000 | pos);

}

//write text to the LCD
void LCDtext(char *data)
{
	while (*data)
	{
		LCD_char(*data);
		data++;
	}
}

// init LCD

void LCD_init(void)
{
	DDRA = 0xFF;  // PORTA as output
	_delay_ms(40);

	// set 4-bit mode
	PORTA = 1<<2;
	PORTA |= 1<<LCD_E;
	_delay_ms(1);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(1);

	PORTA = 1<<2;
	PORTA |= 1<<LCD_E;
	_delay_ms(1);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(1);

	PORTA = 1<<2;
	PORTA |= 1<<LCD_E;
	_delay_ms(1);
	PORTA &= ~(1<<LCD_E);
	_delay_ms(1);

	//set 4-bit mode and 2-line
	LCD_inst (0b00101000);

	//turn on display and cursor
	LCD_inst (0b00001100);

	//clr display
	LCD_inst (LCD_CLR);

}

void Lcd_init_custom(void){
	//lcd_en = 1;
	//lcd_rs = 0;
	DDRA = 0xFF;  // PORTA as output
	_delay_ms(40);	
	
	LCD_inst(0x33);
	LCD_inst(0x32);
	LCD_inst(0x28);
	LCD_inst(0x0C);
	LCD_inst(0x06);
	LCD_inst(0x01);
	
	_delay_ms(20);
	//clr display
	LCD_inst (LCD_CLR);
}
void loadCGRAM(){
	//akses CGRAM
	LCD_inst(0x40);
	
	//Rasteyum Arab
	
	//0x0,0x0,0x1,0x1,0x7,0x4,0x4,0x4		
	//%0,%0,%1,%1,%111,%100,%100,%100
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x1);
	LCD_char(0x1);
	LCD_char(0x7);
	LCD_char(0x4);
	LCD_char(0x4);
	LCD_char(0x4);
	
	//0x0,0x0,0x1b,0x1a,0x3,0x0,0x3,0x6
	//%0,%0,%11011,%11010,%11,%0,%11,%110
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x1b);
	LCD_char(0x1a);
	LCD_char(0x3);
	LCD_char(0x0);
	LCD_char(0x3);
	LCD_char(0x6);
	
	//0x0,0x0,0x11,0x11,0x1f,0x10,0x15,0x0
	//%0,%0,%10001,%10001,%11111,%10000,%10101,%0
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x11);
	LCD_char(0x11);
	LCD_char(0x1f);
	LCD_char(0x10);
	LCD_char(0x15);
	LCD_char(0x0);
	
	//0x0,0x0,0x0,0x8,0xa,0x8,0xf,0x0
	//%0,%0,%0,%1000,%1010,%1000,%1111,%0
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x8);
	LCD_char(0xa);
	LCD_char(0x8);
	LCD_char(0xf);
	LCD_char(0x0);
	
	
	//0x0,0x0,0x0,0x5,0x15,0x5,0x1c,0x0
	//%0,%0,%0,%101,%10101,%101,%11100,%0
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x5);
	LCD_char(0x15);
	LCD_char(0x5);
	LCD_char(0x1c);
	LCD_char(0x0);
	
	
	//0x0,0x0,0x0,0x10,0x0,0x1,0x1e,0x0
	//%0,%0,%0,%10000,%0,%1,%11110,%0
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x10);
	LCD_char(0x0);
	LCD_char(0x1);
	LCD_char(0x1e);
	LCD_char(0x0);
	
	//0x0,0x0,0x14,0x16,0x1f,0x10,0x1,0x3
	//%0,%0,%10100,%10110,%11111,%10000,%1,%11
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x14);
	LCD_char(0x16);
	LCD_char(0x1f);
	LCD_char(0x10);
	LCD_char(0x1);
	LCD_char(0x3);
	
	//0x0,0x0,0x10,0x11,0x13,0x6,0x1c,0x0
	//%0,%0,%10000,%10001,%10011,%110,%11100,%0
	LCD_char(0x0);
	LCD_char(0x0);
	LCD_char(0x10);
	LCD_char(0x11);
	LCD_char(0x13);
	LCD_char(0x6);
	LCD_char(0x1c);
	LCD_char(0x0);
	
	
	//Back to DDRAM
	LCDgoto (1,1);
}