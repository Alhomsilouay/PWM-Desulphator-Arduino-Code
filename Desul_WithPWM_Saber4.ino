
// start : Dec 2021


// rem. Variables shared between ISR functions and normal functions should be declared "volatile". 
// How long does it take to execute an ISR?  2.625 µS to execute, plus whatever the code itself does.

// bitSet(TCCR1B,WGM12); // CTC mode
// bitSet(TCCR1B,CS10); // no prescaling
// bitSet(TIMSK1,OCIE1A); // enable timer 1 compare interrupt
// The Timer/Counter is inactive when no clock source is selected.

// int dutyCycleA;
/*
//PWM definitions
#define SUPPLY_PWM        OCR0B
#define LOAD_PWM        OCR0A
#define PWM_START       TCCR0B |= (1<<CS00)
#define PWM_STOP        TCCR0B &= (~CS00)
#define PWM_SUPPLY_MODE     TCCR0A=(1<<WGM01)|(1<<WGM00)|(1<<COM0B1)
#define PWM_LOAD_MODE     TCCR0A=(1<<WGM01)|(1<<WGM00)|(1<<COM0A1)
#define PWM_RESET       TCCR0A = 0

#define TIMER1_START      TCCR1B = (1<<WGM12)|(1<<CS10)|(1<<CS12)     
#define TIMER1_STOP       TCCR1B = 0
*/

// (timer speed (Hz)) = (Arduino clock speed (16MHz)) / prescaler
// interrupt frequency (Hz) = (Arduino clock speed 16,000,000Hz) / (prescaler * (compare match register + 1))
// compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
// Mode 14 : Period for 16MHz crystal, Top = 16*10^6/(Prescale*Fpwm)-1


#include <avr/io.h>
#include <avr/interrupt.h>
//#include "TimerOne.h" 

#define Timer1A_ON        TCCR1A |= (1 << COM1A1);  // set the bit that enables pwm on PB1 
#define Timer1A_OFF       TCCR1A &= ~_BV(COM1A1);   // clear the bit that disenables pwm on PB1 
#define Timer1B_ON        TCCR1A |= (1 << COM1B1);   // set the bit that enables pwm on PB2 
#define Timer1B_OFF       TCCR1A &= ~_BV(COM1B1);   // clear the bit that disenables pwm on PB2
    
    

// These constants won't change
static int microMHz = 16; // Micro clock frequency

// variables

//const byte digDisCharging_OutPin =  13;  // Command discharging pin 19

const byte digOutPWM_APin        =  9; // digital OutPWM_A pin 15
const byte digOutPWM_BPin        =  10; // digital OutPWM_B pin 16

volatile unsigned int pw_counter; // changed
unsigned int ccounter=0;
volatile unsigned int curICR1 = 1599;
volatile unsigned int curOCR1A = 39;
volatile unsigned int curOCR1B = 399;

//////////////////////////////////////////////////////////////////
void setup()
{
   Serial.begin(115200);
 
   uint8_t sreg = SREG;
   cli();//stop interrupts

   //set timer1 interrupt 
   TCCR1A = 0;// set entire TCCR1A register to 0
   TCCR1B = 0;// same for TCCR1B
   TIMSK1 = 0;
   TCNT1  = 0;//initialize counter value to 0
  
   // set none-inverting mode for OCA1 and OCB1
   Timer1A_ON;  //TCCR1A |= (1 << COM1A1);  // set the bit that enables pwm on PB1 
   Timer1B_OFF; //TCCR1A &= ~_BV(COM1B1);   // clear the bit that disenables pwm on PB2
         
   
   //  Mode 14, Fast PWM mode using ICR1 as TOP
   TCCR1A |= (1 << WGM11);
   TCCR1B |= (1 << WGM12) |(1 << WGM13); 
 
   ICR1   = curICR1;     // Fpwm=10KHz   Tpwm=100 MicroSecs //set top value to ICR1    
   
   OCR1A = curOCR1A;   // (1+Top)*DutyCycle-1 where DutyCycle=On/Off
   OCR1B = curOCR1B;
 
   TIMSK1 |= (1 << TOIE1);  // enable Timer1 overflow interrupt:
   
   TCCR1B |= (1 << CS10); // Start timer : Set CS10 bit so timer runs at clock speed: (no prescaling)

   // Initilise port:  PB1/OC1A  pin as output
   pinMode(digOutPWM_APin      ,OUTPUT);  // Sets the digital pin as output for charging   // DDRB |=(1<<PB1);
   pinMode(digOutPWM_BPin      ,OUTPUT);  // sets the digital pin as output for discharging  // DDRB |=(1<<PB2);
   
   pw_counter=1;
   
   //TCNT1 = 0;
   
   //sei();//allow interrupts
   SREG = sreg;
   
   Serial.println("setup ok");
} 
// --------------------- End setup -----------------

//ISR(TIMER1_COMPA_vect)
ISR(TIMER1_OVF_vect)
{
  switch (pw_counter) 
  {
    case 1:
       {
       }
      break;
    case 10:
       {
         // sets the duty cycle zero
         Timer1A_OFF;     //TCCR1A &= ~_BV(COM1A1);   // clear the bit that disenables pwm on PB1
         //digitalWrite(digOutPWM_APin, LOW);
       }
       break;
    case 11:
       {
         // sets the duty cycle 
         //OCR1B = curOCR1B;
        // Timer1B_ON;   
          PORTB = B00000100;   // digitalWrite(digOutPWM_BPin, HIGH);
         delayMicroseconds(4);
         PORTB = B00000000;   //// digitalWrite(digOutPWM_BPin, LOW);
        // Timer1B_OFF;
           
         //OCR1A = 0;

         /* or 
          *  digitalWrite(digOutPWM_BPin, LOW);
          *  delayMicrosecs()
          */
        }
       break;
    case 30:   
       {
         //OCR1A = curOCR1A;
         //OCR1B = 0;
         Timer1A_ON;             
         pw_counter=0;
       }
       break;
    default:
       {
       }
       break; 
  } // end swith pw_counter   
  pw_counter++; 
}

///////////////////////////////////////////////////////////////////////////////

void loop()
{  
   // The total cycle is 1/333 Hz = 3 ms = 3000 microsec

/*  noInterrupts();
    ccounter=pw_counter;
  interrupts(); 

   if (ccounter==20)
   {
   Serial.println(ccounter);
   }
  */ 
}
// --------------------- End loop -----------------



// End of program End of program End of program


//  SETBIT(PORTC,6);
//   CLEARBIT(PORTC,6)


