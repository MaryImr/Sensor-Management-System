#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <string.h>
#include <avr/sleep.h>
#include "lcdSettings.c"

#define BR_Calc 26
#define COLUMNS_INDICATOR 0x0E
#define COLUMNONE 0X02
#define COLUMNTWO 0X04
#define COLUMNTHREE 0X08
#define ON 1
#define OFF 0
#define FIRSTLINE 0
#define SECONDLINE 1
#define THIRDLINE 2
#define FOURTHLINE 3
#define  LCDCMDEIGHTBIT 0x38
#define LCDCMDDISPLAY 0x0C
#define LCDCMDCURSOR 0x06

#define LCDDISPLAYLINESAFETYLIMIT 3
#define LCDDISPLAYCURSORSAFETYLIMIT 15
#define RESET 0

char Read_Keypad();

// interrupt that wakes mcu up, triggered by external pin when keypad is pressed
ISR(INT0_vect) {
	sleep_disable(); // disable sleep once an interrupt wakes CPU up
	// get row and column from keypad
	char result = Read_Keypad();
	// send character to main logger
	SendData(result);
	// display feedback to user
	lcdWrite(">>", FOURTHLINE);
	SendData('.');
	lcdData(result);
}

void toggleRow(char row, char status) {
	if(status == ON) {
		PORTC = PORTC | (1<<row);
	}
	else if(status == OFF) {
		PORTC = PORTC & ~(1<<row);
	}
}

// function reads the keypad
char Read_Keypad() {
	//which column pin is asserted
	char column = RESET;
	char check = PINC & COLUMNS_INDICATOR;
	if(check == COLUMNONE) {
		column = 1;
	}
	else if(check == COLUMNTWO) {
		column = 2;
	}
	else if(check == COLUMNTHREE) {
		column = 3;
	}

	// we turn off one row at a time until the pin is de-asserted
	char row;
	for (row = 4; row < 7; row++) {
		toggleRow(row, OFF);
		if (!(PINC&(1<<column))) {
			toggleRow(row, ON);
			break;
		}
		toggleRow(row, ON);
	}
	
	if(row == 4) {
		if(column == 1)
		return '3';
		else if(column == 2)
		return '2';
		else if(column == 3)
		return '1';
	}
	else if(row == 5) {
		if(column == 1)
		return '6';
		else if(column == 2)
		return '5';
		else if(column == 3)
		return '4';
	}
	else if(row == 6) {
		if(column == 1)
		return '9';
		else if(column == 2)
		return '8';
		else if(column == 3)
		return '7';
	}
	else if(row == 7) {
		if(column == 1)
		return '#';
		else if(column == 2)
		return '0';
		else if(column == 3)
		return '.';
	}
}

// transmits a single byte of data to the sensor
void SendData(unsigned char packetOut) {
	while(!(UCSR1A & (1<<UDRE1)))
	_delay_ms(100);
	UDR1 = packetOut;
}

char dataStore[128];
char dataStorePointer = RESET;
char charactersDisplayed = RESET;
char displayLine = 1;
char charsReceived = RESET;
char received_data_size = RESET;

// receive from logger
ISR(USART1_RX_vect) {
	sleep_disable(); // disable sleep on interrupt
	
	char receivedData = UDR1;
	if(receivedData != '>'){
		dataStore[dataStorePointer++] = receivedData;
	}
	else{
		// reset buffer pointer
		dataStore[dataStorePointer] = '\0';
		dataStorePointer = 0;
		// if first character is '<', clear the screen and prep for mem dump
		if(dataStore[0] == '<'){
			lcdClr();
			lcdWrite(dataStore, FIRSTLINE);
			charactersDisplayed = RESET;
			displayLine = 1;
		}
		// memory dump or last entry
		else{// memory dump specific
			lcdData(dataStore[1]);
			lcdData(dataStore[2]);
			lcdData(dataStore[3]);
			// used to rollover the lcd cursor
			charactersDisplayed += 3;
			if(charactersDisplayed > LCDDISPLAYCURSORSAFETYLIMIT){
				charactersDisplayed = RESET;
				lcdLineSelect(++displayLine);
				if(displayLine > LCDDISPLAYLINESAFETYLIMIT){
					lcdClr();
					displayLine = RESET;
				}
			}
		}
	}
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

void keyBoardSettings() {
	DDRC = 0xF0; // keypad inputs
	PORTC = 0xF0; // pull up
	//interrupt
	EIMSK = 0x01; // normal mode
	EICRA |= (0x02);
}

void lcdSett() {
	DDRE = 0xFF; // data pins
	DDRD = 0xE0; // RS RW E 0 0 0 0 0

	lcdCommand(LCDCMDEIGHTBIT); // 8 bit mode
	lcdCommand(LCDCMDDISPLAY); // display on cursor off
	lcdCommand(LCDCMDCURSOR); // shift cursor to the right
}

void configure() {
	usartSettings();
	lcdSett();
	keyBoardSettings();
}

int main(void) {
	configure();
	while (1) {
		sei();// waits for user or sensor interrupts
		sleep_enable(); // sleep mode
		sleep_cpu(); // put CPU to sleep
	}
}