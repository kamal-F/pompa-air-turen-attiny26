/*
* pompa_air_1.c
*
* Created: 2/1/2014 19:12:30
*  Author: kxm
*/

/**
Sonar interfacing:
1. Send high impulse to Trig input for minimum 10us
2. Sonar automatically sends eight 40kHz inpulses
3. Sonar rises high on Echo output and then after some time drops
output to low (can take a while on long distances! - must include timeouts)
4. Based on output time difference deltaT = lowT-highT calculate:
distance = [ deltaT * sound_speed(340m/s) ] / 2
5. Make a delay before starting the next cycle to compensate for late echoes
*/

/*
ADC diganti HC-SR04 ultra sonic range finder
trigger by PB3 (ICno4)
echo by interrupt INT0 PB6 (ICno9)
**/

#ifndef F_CPU
#define F_CPU 8000000UL // 8 MHz clock speed
#endif

#define DEBOUNCE_TIME 20

#define Hidup "Hidup"
#define Mati "Mati"
#define treshold 5	//jarak ke sensor tinggal 5cm lagi

#define SET(port,pin) PORT ## port |= (1<<pin)
#define CLEAR(port,pin) PORT ## port &= ~(1<<pin)

#define INSTR_PER_MS 1000// wprescalar 8000/8                // instructions per millisecond (depends on MCU clock, 8MHz current)
#define MAX_RESP_TIME_MS 200      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 1 // echo cancelling time between sampling
//
#define  max_ticks 200000 // MAX_RESP_TIME_MS * INSTR_PER_MS

#define tinggiTuren 120	//1.2m=120cm

#include <avr/io.h>
#include <util/delay.h>
#include "lcd_lib.h"
#include <avr/interrupt.h>
#include <stdlib.h>

volatile uint8_t statusPompa=0;	//led pompa
volatile uint8_t statusOtomatis=0;	//led otomatis

uint8_t debounceCount;
uint8_t sisatinggiTuren;

volatile long result = 0;
volatile unsigned char running = 0;
volatile uint32_t timerCounter = 0;

void showLCDPompa(void);
void showLCDOtomatis(void);
void sonar(void);

//isr timer0 overflow, passes 255
//utk button
ISR(TIMER0_OVF0_vect){
	
	//debounce delay (30/20) eksekusi/detik
	
	if(debounceCount>0){
		debounceCount=debounceCount-1;
		return;
	}
	
	if(bit_is_clear(PINB,PB5)){
		debounceCount=DEBOUNCE_TIME;
		statusPompa=!statusPompa;
		showLCDPompa();
	}
	
	if(bit_is_clear(PINB,PB4)){
		debounceCount=DEBOUNCE_TIME;
		statusOtomatis=!statusOtomatis;
		showLCDOtomatis();
	}
	
	if(statusPompa){
		//PORTB=0b00110001;
		SET(B,0);
	}
	else{
		//PORTB=0b00110000;
		CLEAR(B,0);
	}
	
	if(statusOtomatis){
		SET(B,1);
	}
	else{
		CLEAR(B,1);
	}
	
	//read sensor
	if (running == 0) { // launch only when next iteration can happen
		// create a delay between tests, to compensate for old echoes
		_delay_ms(DELAY_BETWEEN_TESTS_MS);
		sonar(); // launch measurement!
	}
}

//timer1 utk sensor
ISR(TIMER1_OVF1_vect){
	timerCounter++; // count the number of overflows
	
	// dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
	uint32_t ticks = timerCounter * 256 + TCNT1;
	
	if (ticks > max_ticks) {
		// timeout
		TCCR1B = (0<<CS13)|(0<<CS12)|(0<<CS11)|(0<<CS10); //timer off
		running=0;
		result = -1; // show that measurement failed with a timeout (could return max distance here if needed)
	}
}

//check echo
SIGNAL(INT0_vect){
	// interrupt for INT0 pin, to detect high/low voltage changes
	
	//cek PB6 int0
	if(running){
		//if high then nyalakan timer
		TCCR1B = (0<<CS13)|(1<<CS12)|(0<<CS11)|(0<<CS10);//timer on 8mhz/prescalar8
		timerCounter = 0;
		TCNT1 = 0; // reset timer counter
		running=0;
	}
	else{
		//matikan timer
		TCCR1B = (0<<CS13)|(0<<CS12)|(0<<CS11)|(0<<CS10);//timer off
		result = (timerCounter * 256 + TCNT1) / 58;
		
		//check result if otomatis? treshold (jarak ke sensor)
		if(statusOtomatis && result <= treshold){
			statusPompa=0;
			CLEAR(B,0);
		}
		
	}
}

// generate an impulse for the Trig input (starts the sonar)
void sonar() {
	//trigger PB3 (ICno4) PING
	
	CLEAR(B,3);	// clear to zero for 1 us
	_delay_us(1);//1
	
	SET(B,3);// set high for 10us
	running = 1;  // sonar launched
	_delay_us(10);//10
	
	CLEAR(B,3);// clear
}

void initSonar(){
	// ------------------- ultrasonic init code --------------------
	// turn on interrupts for INT0, connect Echo to INT0
	
	MCUCR |=(1<<ISC00)| (0 << ISC01);// enable interrupt on any fall/down
	
	GIMSK|= (1 << INT0); // Turns on INT1 //pd sistem yg lain *GICR |= (1 << INT1);
	
	TCCR1B |= (0<<CS13)|(0<<CS12)|(0<<CS11)|(0<<CS10);//timer off
	TCNT1= 0; // reset counter to zero
	TIMSK |=(1<<TOIE1);//Timer/Counter1 Overflow Interrupt Enable
	
	//interrupts send by HC-SR04 automatically, hi at PB6
	
}

void initTimer(){
	// enable timer overflow interrupt for Timer0
	TIMSK=(1<<TOIE0);

	// set timer0 counter initial value to 0
	TCNT0=0x00;

	// start timer0 with /1024 prescaler , attiny26 page 66/182
	TCCR0 = (1<<CS02) | (1<<CS00);
	
	//jadi triger per detik
	//CPU / timer0(255) / prescaler
	//(8,000,000 / 255 / 1024 =  30).
	
}

void showLCDPompa(void){
	LCD_inst (LCD_CLR);
	LCDtext ("Pompa: ");
	
	if(statusPompa){
		LCDtext(Hidup);
	}
	else{
		LCDtext(Mati);
	}
	//_delay_ms(2000);
}
void showLCDOtomatis(void){
	LCD_inst (LCD_CLR);
	LCDtext ("Otomatis: ");
	
	if(statusOtomatis){
		LCDtext(Hidup);
	}
	else{
		LCDtext(Mati);
	}
	//_delay_ms(2000);
}



int main(void)
{
	
	DDRA = 0xFF; //LCD
	
	DDRB=0b10001111;//PB 4,5,6 as input,	PB0 as output to pompa
	//1=output and 0=input.
	//INT0 di PB6
	
	PORTB=0b00110000;//pull up resistor at PB 4, 5
	
	initTimer();
	initSonar();
	Lcd_init_custom();
	loadCGRAM();
	
	sei();   // Enable Global Interrupts
	
	char aNumberAsString[4];
	uint8_t sensor_result_disp=0;
	uint8_t DisplayTuren=0;
	uint8_t DisplayTurenSisa=0;
	
	while(1)
	{
		
		//Logo ibu
		LCD_inst (LCD_CLR);
		LCDgoto (7,0);
		LCD_char(0x00);
		LCD_char(0x01);
		LCD_char(0x03);
		LCD_char(0x04);
		LCD_char(0x05);
		LCD_char(0x06);
		LCD_char(0x07);
		LCDgoto (2,1);
		LCDtext ("RASTEYUM");
		_delay_ms(4000);
		
		//status pompa
		showLCDPompa();
		_delay_ms(2000);
		
		//status otomatis
		showLCDOtomatis();
		_delay_ms(2000);
		
		
		
		//reset zero display
		sensor_result_disp=0;
		DisplayTuren=0;
		
				
		sensor_result_disp=result;
		
		//DisplayTuren
		sisatinggiTuren = tinggiTuren - sensor_result_disp;
		
		itoa(sisatinggiTuren, aNumberAsString, 10);
		LCD_inst (LCD_CLR);
		LCDtext("Turen: ");
		LCDtext(aNumberAsString);
		LCDtext(" %");
		LCDgoto (0,1);
						
		if(sisatinggiTuren>7){DisplayTuren=1;}
		if(sisatinggiTuren>19){DisplayTuren=2;}
		if(sisatinggiTuren>29){DisplayTuren=3;}
		if(sisatinggiTuren>39){DisplayTuren=4;}
		if(sisatinggiTuren>49){DisplayTuren=5;}
		if(sisatinggiTuren>59){DisplayTuren=6;}
		if(sisatinggiTuren>69){DisplayTuren=7;}
		if(sisatinggiTuren>79){DisplayTuren=8;}
		if(sisatinggiTuren>89){DisplayTuren=9;}
		if(sisatinggiTuren>95){DisplayTuren=10;}
		
		//isi turen
		for(uint8_t j=0;j<DisplayTuren;j++){
			LCD_char(0xff);//kotak
		}
		
		//sisa
		DisplayTurenSisa=10-DisplayTuren;
		for(uint8_t j=0;j<DisplayTurenSisa;j++){
			LCD_char(0x2e);//titik
		}
		
		_delay_ms(2000);
		
	}
}