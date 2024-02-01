#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include <avr/sleep.h>
#define F_CPU 4000000
#include <util/delay.h>
#include "lcdSettings.c"
#include "CRC.h"
#define BR_Calc 26
char reset = 0;
char TOS = 0;
char TOS_STATE = 0;
#define EMPTY  0
#define FULL   1

int sensors[4];
#define TEMPERATURE 0 
#define MOISTURE 1
#define WATERLEVEL 2
#define BATTERY 3
#define LOWBATTERYLIMIT 0x14
#define LCDCMDEIGHTBIT 0x38
#define LCDCMDDISPLAY 0x0C
#define LCDCMDCURSOR 0x06
#define TENSECLIMITCOUNT 26464

#define COMMAND 0x80
#define RESET  0x00  
#define REPEAT 0xE0  
#define ACNKOWLEDGE 0x40  
#define LOG 0x20  
#define TOPBITSREMOVER 0xE0

#define FASTINVPWM 0x78
#define TENTOFIVEDIVISOR 32
#define INTERRUPENABLE 0x04
#define PRESCALE1024 0x05
#define PRESCALE64 0x06
#define PRESCALE1 0x01

#define FIRSTLINE 0
#define SECONDLINE 1
#define THIRDLINE 2
#define FOURTHLINE 3

// transmits a single byte of data to the sensor
void Transmit(unsigned char packetOut) {
	while(!(UCSR1A & (1<<UDRE1)))
	_delay_ms(10);
	UDR1 = packetOut;
}

void halfSecDel() {
	_delay_ms(50);
}

// receive data
ISR(USART1_RX_vect) {
	sleep_disable(); // disable sleep once an interrupt wakes CPU up
	unsigned char data = UDR1;
	
	// crc check data and send repeat if necessary
	if(!CRC_CHECK3(data)){
		Transmit(CRC3(REPEAT));
		halfSecDel();
	}
	
	else {
		data = data & TOPBITSREMOVER;
	
		// if reset
		if(data == RESET){
			if(reset)
				setup();
			// set first reset bit
			reset = 1;
		
			// empty stack
			TOS = 0;
			TOS_STATE = EMPTY;
		
			// acknowledge
			Transmit(ACNKOWLEDGE);
		}
		else if (data == REPEAT){
			// resend tos
			Transmit(TOS);
		}
		else if (data == ACNKOWLEDGE){
			// empty stack, used as a flag when sending multiple sensors, sending waits for stack to empty
			TOS = 0;
			TOS_STATE = EMPTY;
		}
	} 
}

// sets the adc conversion flag and wakes mcu up
char gettingSensorValue = 0;
ISR(ADC_vect){
	sleep_disable();
	gettingSensorValue = 1;
}

int tenToFiveBitADCConverter() {
	//divided by 32 to map 10 bit number to 5 bits
	return ADC/TENTOFIVEDIVISOR;
}

// reads a single pin sensor and converts to digital then returns the 10 bit number
int read_sensor(char type){
	gettingSensorValue = 0;
	ADMUX = ADMUX & TOPBITSREMOVER; // clear bits
	ADMUX = ADMUX | type; // select the pin to read
	
	ADCSRA |= (1<<ADSC); // starts conversion
	// sleep and await for conversion to end, using while in case mcu wakes up
	// due to another interrupt
	while(1){
		sleep_enable(); // arm sleep mode
		sleep_cpu(); // put CPU to sleep
		if(gettingSensorValue)
			break;
	}

	return tenToFiveBitADCConverter();
}

// initializes the analog to digital components
void ADCInitialization(){
	// sets inputs and outputs
	DDRF = 0xF0;
	PORTF = 0xF0;
	
	ADCSRA = ADCSRA | (1<<ADEN); // enable ADC
	ADCSRA = ADCSRA | (1<<ADIE); // enable interrupts
	ADCSRA = ADCSRA | PRESCALE64; 
	
	// to maintain adc clk frequency between 50kHz and 200kHz
	ADMUX = 0x40; // select vref = 5V 0100
	
	// initiate the first conversion to ensure that subsequent conversions are 13 clock cycles
	read_sensor(TEMPERATURE);
}

// reads all the 4 connected sensors and write their values to the related buffers
void receiveAllSensorsValues(){
	sensors[TEMPERATURE] = read_sensor(TEMPERATURE);
	sensors[MOISTURE] = read_sensor(MOISTURE);
	sensors[WATERLEVEL] = read_sensor(WATERLEVEL);
	sensors[BATTERY] = read_sensor(BATTERY);
}

// prints the sensor values from buffer
void displaySensorReadings(){
	lcdClr();
	char printer[80];
	
	_delay_ms(200);
	if(sensors[BATTERY] < LOWBATTERYLIMIT){ // if less than 3.2 volts
		lcdWrite("<~1 Low battery!", FIRSTLINE);
	}
	else{
		sprintf(printer, "<T=%x   M=%x\nW=%x   B=%x", sensors[TEMPERATURE], sensors[MOISTURE], sensors[WATERLEVEL], sensors[BATTERY]);
		lcdWrite(printer, FIRSTLINE);
	}
}

// sends all sensor values from buffer to central logger
void transmitToSensor(){
	lcdClr();
	lcdWrite("<~0Transmitting to\n logger", 0); 
	char sensorID;
	for(sensorID = 0; sensorID < 4; sensorID++){
		//sensor order - T, M, W, B
		TOS = COMMAND | LOG | sensors[sensorID];
		TOS_STATE = FULL;
		
		// sends the data packet
		Transmit(TOS);
		_delay_us(10);
		
		// sends the log request and crc bits
		char updatedLOG = LOG | CRC11(LOG, TOS);
		Transmit(updatedLOG);
		
		// awaits an ack by waiting for tos to empty up
		while(TOS_STATE == FULL){
			sleep_enable(); // arm sleep mode
			sleep_cpu(); // put CPU to sleep
		}
	 }
	// print the sensor values to the lcd
	displaySensorReadings();
}

// sets the pwm settings for the motor speed
void motorSpeedInit(){
	char dc;
	if(sensors[MOISTURE] == 0) {
		dc = 80;
	}
	else if(sensors[MOISTURE] == 0x1F) {
		dc = 20;
	}
	// inverting ocr
	char ocr = -1 * ((dc * 256 / 100) - 255);
	OCR0 = ocr;
	TCCR0 |= PRESCALE1; 
}

// starts the motor
void sprinklerStart(){
	motorSpeedInit();
	// sets the direction and turns on the motor
	PORTB = PORTB | (0x40);
	PORTB = PORTB & ~(0x80);
	TCCR0 = TCCR0 | 1; // start pwm, unnecssary but for speeding up simulation
}

// stops the motor
void sprinklerStop(){
	PORTB = PORTB & ~0x40 & ~0x80;
	TCCR0 = TCCR0 & 0;
}

ISR(TIMER3_OVF_vect){
	// stop watering
	TCCR3B = 0x00; // stopping timer
	sprinklerStop();
}

ISR(TIMER1_OVF_vect){
	TCCR1B = 0x00; // stopping timer
	
	// read sensors
	sei();
	receiveAllSensorsValues();
	transmitToSensor();
	
	// start watering
	TCNT3 = 52948; // setting timer 3 to run for 5 seconds
	TCCR3B = PRESCALE1024; // starting timer3 with scaler = 1024
	sprinklerStart();
	
	TCNT1 = TENSECLIMITCOUNT; // setting counter1 to 26464 for 10 seconds
	TCCR1B = PRESCALE1024; // starting timer with scaler = 1024
}

void timerInit(){
	// watering interval sensing interval (since they both use the same timer of 10 seconds)
	TCNT1 = TENSECLIMITCOUNT; // setting counter1 to 26464 for 10 seconds
	TIMSK = INTERRUPENABLE; // timer 1 interrupt
	ETIMSK = INTERRUPENABLE; // timer 3 interrupt
	TCCR1B = PRESCALE1024;
}

void init_motor(){
	DDRB = 0xFF; // setting in1 and in2 to output, determines direction of dc motor
	
	// using timer0 for pwm
	TCCR0 = FASTINVPWM; // enable fast inverting pwm
}

void usartSettings() {
	// BlueTooth
	UCSR1C = (1<<UCSZ11); // setting data width to 8
	UCSR1C |= (1<<UCSZ10);
	UBRR1H = (BR_Calc>>8); // setting baud rate to 9600 by setting UBBR
	UBRR1L = BR_Calc;
	UCSR1B = (1<<TXEN1); // enable transmitter, receiver, and receive and transmit complete interrupts
	UCSR1B |= (1<<RXCIE1);
	UCSR1B |= (1<<RXEN1);
	sei();
}

void lcdSett() {
	DDRE = 0xFF; // data pins
	DDRD = 0xE0; // RS RW E 0 0 0 0 0

	lcdCommand(LCDCMDEIGHTBIT); // 8 bit mode
	lcdCommand(LCDCMDDISPLAY); // display on cursor off
	lcdCommand(LCDCMDCURSOR); // shift cursor to the right
}

void setup() {
	
	usartSettings();
	lcdSett();
	ADCInitialization(); // initialize ADC
	
	// set sensors
	receiveAllSensorsValues();
	displaySensorReadings();
	
	init_motor();
}

void initialization() {
	setup();
	while(1){
		sleep_enable(); // arm sleep mode
		sleep_cpu(); // put CPU to sleep
		if(reset) {
			break;
		}
	}
	timerInit();
}

int main(void)
{
	initialization();
	while (1){
		sleep_enable(); // arm sleep mode
		sleep_cpu(); // put CPU to sleep
	}
}