#define F_CPU 4000000
#include <util/delay.h>

#define LINEONE 0X80
#define LINETWO 0Xc0
#define LINETHREE 0X94
#define LINEFOUR 0XD4

#define LCDENABLE 0X80
#define LCDRS 0X20
#define LCDCLEAR 0X01

void lcdCommand(unsigned char);
void lcdData(unsigned char);
void lcdSend(unsigned char);
void lcdWrite(const char*, char);

void tenSecDel() {
	_delay_us(1000);
}

void lcdLineSelect(char line){
	switch (line){
		case 0:
		lcdCommand(LINEONE);
		break;
		case 1:
		lcdCommand(LINETWO);
		break;
		case 2:
		lcdCommand(LINETHREE);
		break;
		case 3:
		lcdCommand(LINEFOUR);
		break;
	}
	tenSecDel();
}

void lcdClr() {
	lcdCommand(LCDCLEAR); // clear display screen
	tenSecDel();
}

void lcdCommand(unsigned char command) {
	PORTD &= ~(LCDRS); // setting rs to 0
	lcdSend(command); // 0000 0000 RS=0, RW=0
}

void lcdData(unsigned char data) {
	PORTD |= (LCDRS); // setting rs to 1
	lcdSend(data); // 0010 0000
}

void lcdSend(unsigned char data) {
	PORTE = data;
	PORTD |= LCDENABLE; // 1000 0000 send pulse to E
	_delay_us(100);
	PORTD &= ~(LCDENABLE); // turn E pulse off
	_delay_us(100);
}

void lcdWrite(const char sentence[], char l) {
	lcdLineSelect(l);
	int index;
	for(index=1; sentence[index]!='\0'; index++) {
		(sentence[index] == '~') ? lcdData(sentence[++index] - '0') : (sentence[index] == '\n') ? lcdLineSelect(++l) : lcdData(sentence[index]);
	}
}
