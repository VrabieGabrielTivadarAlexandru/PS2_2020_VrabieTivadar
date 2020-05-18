#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MAXVAL_TIMER1 32767
#define timer0_10ms_pres1024 156

#define kp 1
#define ki 1
#define kd 0.25

#define ROSU 1
#define ALBASTRU 2
#define GALBEN 3

uint8_t err_prec=0;
int I=0;

void timer0_setup_pres1024(){
	TCCR0B=(1<<CS02)|(1<<CS00);
}

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

uint8_t val_senzori_linie(){
	int val=PIND&0XF8;
	return val>>3;
}

short int error(){
	int val=val_senzori_linie(),err=-2;
	while(val){
		if(!(val&1)) break;
		else err++;
		val>>=1;
	}
	if(val==-2) return 0;
	else return err;
}

int pid(){
	short int err=error(),D=err-err_prec;
	I+=err;
	err_prec=err;
	return kp*err+kd*D+ki*I;
}

void control(){
	int comanda=pid(),stanga=100,dreapta=100;
	if(comanda<0) dreapta-=comanda;
	else if(comanda>0) stanga-=comanda;
	curba_var(stanga,dreapta);
}

void adc_init()
{
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
	ADCSRA |= (1 << ADEN); //enable ADC
	ADMUX |= (1 << REFS0);
	ADCSRA |= (1 << ADSC); //ADC start conversion
}

int read_adc(int channel)
{
	ADMUX &= ~(1 << channel);
	ADMUX |= channel;//select chanel AO to A5
	ADCSRA |= (1 << ADSC); //start conversion
	while (ADCSRA & (1 << ADSC)); //wait while adc conversion are not updated
	return ADCW; //read and return
}

int detectie_culoare(){
	int valmax=0,indmax=0;
	for(int i=1;i<4;i++){
		int val=0;
		PORTC|=(1<<i);
		if(val>valmax) {
			valmax=val;
			indmax=i;
		}
		PORTC&=~(1<<i);
	}
	return indmax;
}

void actiune(int culoare){
	switch(culoare){
		case ROSU: stop();	//stai 3 secunde si mergi in fata
			_delay_ms(3000);
			fata();
			break;
		case ALBASTRU: fata();	//continua
			break;
		case GALBEN: curba_var(100,80);		//fa o curba la dreapta timp de o secunda
			_delay_ms(1000);
			break;
	}
}

int main() {
	cli();
	timer1_setup_pwm();
	timer0_setup_pres1024();
	DDRB = 0xFF;
	DDRC = 0x70;
	sei();
	stop();
	fata();
	while (1) {
		TCNT0=0;
		control();
		while(TCNT0<timer0_10ms_pres1024){}	//10ms perioada de esantionare
	}
	return 0;
}