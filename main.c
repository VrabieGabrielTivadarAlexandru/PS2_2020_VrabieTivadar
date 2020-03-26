#include <avr/io.h>
#include <avr/interrupt.h>

#define FREQ 16000000		//frecventa microcontrollerului
#define BAUD 9600			//baud rate
#define UBRR FREQ/16/BAUD-1	//valoarea ce trebuie incarcata in registrul UBRR0, e in jur de 206
#define MAXVAL_TIMER1 32767
#define TIMEOUT_VAL 11764	//340m/s => 0.034cm/us => us_100cm=(100cm)/(0.034cm/us)=2941.17 => TIMEOUT_VAL= us_100cm*2*2 -inmultim cu 4 pt ca timerul se incrementeaza de 2 ori per microsecunda si luam in considerare si timpul de intoarcere a undei ultrasonice

#define WAIT 0
#define FREE 1

#define START_PAUSE 0
#define SEARCHING 1
#define MOVING_TOWARDS_OBSTACLE 2
#define CYCLE_ENDED 4

#define OVF 0
#define NONOVF 1

//sonar
volatile uint32_t timp_us, timp_initial, timp_final_delay,timp_final,timeout;
volatile uint8_t distanta,contor_obj=0,timeout_case;
volatile uint8_t distanta_arr[4];
volatile uint8_t stare_sonar=FREE,stare_robot=START_PAUSE;

void EXTINT0_IR_SETUP(){
	EICRA|=(1<<ISC00);	//se seteaza intreruperea externa INT0 sa se activeze pe orice front (schimbare de culoare a suprafetei)
	EIMSK|=(1<<INT0);
}

void EXTINT1_ULTRASONIC_SETUP(){
	EICRA|=(1<<ISC10)|(1<<ISC11);
	EIMSK|=(1<<INT1);
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

void fata()
{
	OCR1A = 32767;
	OCR1B = 32767-5000;
	PORTB &= ~0x30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void spate()
{
	OCR1A = 32767-5000;
	OCR1B = 32767;
	PORTB |= 0x30;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0);
}
void stanga()
{
	OCR1A = 20000;
	OCR1B = 20000;
	PORTB |=0x10;
	PORTB&=~0x20;
	TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
	TCCR1A |= (1<<COM1A0)|(1 << COM1A1) | (1 << COM1B1);
}

void dreapta()
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

uint8_t char_to_num(uint8_t c) {
	return c - '0';
}

void delay_microsecunde(uint16_t microsecunde) {
	uint16_t timp_final_delay = TCNT1 + microsecunde * 2;			//valoarea la care trebuie sa ajunga TCNT1 la finalul delayului, nu incurca pwm-ul
	if (timp_final_delay>MAXVAL_TIMER1){		//masura pentru gestionarea situatiei de overflow
		timp_final_delay-=MAXVAL_TIMER1;
	}
	while (TCNT1 != timp_final_delay) {        //timerul 1 cu prescaler de 8, numara 2 cicluri de tact per microsecunda

	}
}

int main() {
	cli();
	timer1_setup_pwm();
	EXTINT0_IR_SETUP();
	EXTINT1_ULTRASONIC_SETUP();
	DDRB = 0xFF;
	for(uint16_t i=0;i<30000;i++){		//stai 3 secunde
		delay_microsecunde(100);
	}
	sei();
	stanga();
	stare_robot=SEARCHING;
	while (1) {
		if(stare_robot==CYCLE_ENDED) {
			delay_microsecunde(100);
			stanga();
			stare_robot=SEARCHING;
			stare_sonar=FREE;
		}
		else if((stare_sonar==FREE)&&((stare_robot==SEARCHING)||(stare_robot==MOVING_TOWARDS_OBSTACLE))){	//folosim sonarul doar cand cautam un obiect
			PORTB &= ~(0X01);																				//sau cand ne miscam spre un obiect, pentru a-l mentine in fata noastra
			delay_microsecunde(2);
			PORTB |= 0X01;
			delay_microsecunde(11);
			PORTB &= ~(0X01);
			stare_sonar=WAIT;
		}
		
	}
	return 0;
}


ISR(INT1_vect){
	timp_initial = TCNT1;
	while (PIND & 0X08) {

	}
	stare_sonar=FREE;
	timp_final = TCNT1;
	if (timp_final<timp_initial) {						//detectie overflow TCNT1; e imposibil pt aplicatia curenta ca timp_final>timp_initial si sa se fi produs overflow al TCNT1;
		timp_final = MAXVAL_TIMER1 - timp_initial + timp_final;		//se recalculeaza timp_final,va fi de fapt intervalul de timp insusi scurs;
		timp_initial = 0;								//timp_initial se pune pe 0;
	}
	timp_us = (timp_final - timp_initial) / 2;
	timp_us = (timp_us * 34) / 1000 / 2;
	distanta = timp_us;
	delay_microsecunde(50);
	if (distanta<50)
	{
			fata();
			stare_robot=MOVING_TOWARDS_OBSTACLE;
	} 
	else
	{
		stanga();
		stare_robot=SEARCHING;
	}
}

ISR(INT0_vect){
	if(!(PIND&0x04)){							//daca s-a detectat o suprafata alba, adica daca s-a iesit din ring
		spate();
		for(uint16_t i=0;i<5000;i++){		//mergi in spate 2 secunde
			delay_microsecunde(100);
		}
		stop();
		stare_robot=CYCLE_ENDED;
	}
}
