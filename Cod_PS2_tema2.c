#include <avr/io.h>
#include <avr/interrupt.h>

#define MAXVAL_TIMER1 32767

// setare timer1 pt PWM
void timer1_setup_pwm() {			//timer 1 lucreaza in modul Fast PWM, numarand pana la valoarea din ICR1;
	TCNT1 = 0;
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) |(1<<WGM11);
	TCCR1B = (1<<WGM13)|(1 << WGM12) | (1 << CS11);
	ICR1=MAXVAL_TIMER1;
	OCR1A = 0;
	OCR1B = 0;
}

void curba_var(unsigned int motor_st_fact,unsigned int motor_dr_fact){	//cei 2 factori iau valori intre 0 si 100, robotul se va deplasa inainte dar cu vitezele rotilor variabile prin cei 2 factori
	OCR1A = (32767/100)*motor_dr_fact;
	OCR1B = (32767/100)*motor_st_fact;
	PORTB &= ~0x30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void fata()
{
	OCR1A = 32767;
	OCR1B = 32767;
	PORTB &= ~0x30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void spate()
{
	OCR1A = 32767;
	OCR1B = 32767;
	PORTB |= 0x30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0);
}
void stanga_peloc()
{
	OCR1A = 20000;
	OCR1B = 20000;
	PORTB |=0x10;
	PORTB&=~0x20;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1<<COM1A0)|(1 << COM1A1) | (1 << COM1B1);
}

void dreapta_peloc()
{
	OCR1A = 20000;
	OCR1B = 20000;
	PORTB |=0X20;
	PORTB&=~0x10;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1<<COM1B0)|(1 << COM1B1);
}

void stop(){
	OCR1A=0;
	OCR1B=0;
	PORTB&=~0X30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
}


int main() {
	cli();
	timer1_setup_pwm();
	DDRB = 0xFF;
	sei();
	stop();
	while (1) {

	}
	return 0;
}