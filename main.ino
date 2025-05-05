//Monitor the water levels in a reservoir and print an alert when the level is too low
//Monitor and display the current air temp and humidity on an LCD screen.
//● Start and stop a fan motor as needed when the temperature falls out of a specifed
//range (high or low).
//● Allow a user to use a control to adjust the angle of an output vent from the system
//● Allow a user to enable or disable the system using an on/of button
//● Record the time and date every time the motor is turned on or of. This information
//should be transmitted to a host computer (over USB)

#include <Wire.h> 
#include <Adafruit_Sensor.h> 
#include <avr/interrupt.h>
#include <avr/io.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20
#define LED_DIS PB1 //D52 
#define LED_IDLE PB3 //D50
#define LED_RUN PB2 //D51 
#define LED_ERR PB0 //D53 
#define BTN_PIN PD3 //D18 INT1
#define FAN_PIN PC2 //D35 OC1A 

#define VENT_CH 0 //A0 
#define WATER_CH 1 //A1 
#define WATER_THR 650

#define TEMP_HI 30.0
#define TEMP_LO 27.0
#define FAST_MS 500UL
#define SLOW_MS 60000UL

//low level pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portB     = (unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (unsigned char *) 0x24;

volatile unsigned char *port_d = (unsigned char*) 0x2B; 
volatile unsigned char *ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char *pin_d  = (unsigned char*) 0x29; 

volatile unsigned char *port_e = (unsigned char*) 0x2E; 
volatile unsigned char *ddr_e  = (unsigned char*) 0x2D; 
volatile unsigned char *pin_e  = (unsigned char*) 0x2C; 

// Timer Pointers
volatile unsigned char *myTCCR1A  =(unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  =(unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  =(unsigned char *) 0x82;
volatile unsigned int  *myICR1   =(unsigned int *)0x86;
volatile unsigned int  *myOCR1A  =(unsigned int *)0x88;

//this is from water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//global objects
LiquidCrystal LCD(12,11,5,4,3,2);
Stepper ventStepper(200, 23,27,25,29);  
DHT dht(45, DHT11);                 
RTC_DS1307 rtc;

enum CoolerState : uint8_t { DISABLED, IDLE, RUNNING, ERROR };

//global state default disabled and global start button
volatile CoolerState g_state = DISABLED;
volatile bool g_btn = false;

void U0init(int baud)
{
    unsigned int tbaud=(F_CPU/16/baud)-1;
    *myUBRR0  = tbaud;
    *myUCSR0A = 0x20;
    *myUCSR0B = 0x18;
    *myUCSR0C = 0x06;
}
void U0putchar(unsigned char c){
    while(!(*myUCSR0A&TBE)); 
    *myUDR0=c;
}
void U0printString(const char* s){
    while(*s){
        U0putchar(*s++);
    }
}
void U0printNumber(int n){
    char buf[12]; 
    itoa(n,buf,10); 
    U0printString(buf);
}

//================timestamps==================================
void logTime(){
    DateTime now = rtc.now();
    U0putchar('[');
    U0printNumber(now.hour());  
    U0putchar(':');
    if(now.minute()<10){
        U0putchar('0');
    }
    U0printNumber(now.minute());
    U0putchar(':');
    if(now.second()<10){
        U0putchar('0');
    }
    U0printNumber(now.second());
    U0putchar(']');
}

//setState() turns on one LED and prints a letter
void setState(CoolerState s){
    g_state=s;
    *portB &= ~0b00011110;
    switch(s){
      case DISABLED:*portB|=(1<<LED_DIS);
          break;
      case IDLE: *portB|=(1<<LED_IDLE);
          break;
      case RUNNING: *portB|=(1<<LED_RUN);
          break;
      case ERROR: *portB|=(1<<LED_ERR);
          break;
    }
    logTime(); 
    U0putchar(' '); 
    U0putchar("DIRE"[s]); 
    U0putchar('\n');
}
//================button=====================================

void initStartButtonISR() {
    DDRD &= ~(1<<BTN_PIN);
    PORTD |= (1<<BTN_PIN);
    EICRA |= (1<<ISC11);
    EIMSK |= (1<<INT1);
}

ISR(INT1_vect) {
  g_btn = true;
}
//================button=====================================

//==================adc drivers=================================
void adc_init() //write your code after each commented line and follow the instruction 
{
    *my_ADMUX = 0b01000000; //AVCC ref, right‑adj, ch0
    *my_ADCSRA = 0b10000111; //enable, presc 128
    *my_ADCSRB &= 0b11110111; //MUX5=0 

}

unsigned int adc_read(unsigned char ch) //work with channel 0
{
    *my_ADMUX  = (*my_ADMUX & 0b11111000) | (ch & 0x07);
    *my_ADCSRA |= 0b01000000;      
    while(*my_ADCSRA & 0b01000000);
    return *my_ADC_DATA;
}
void fanInit(){
    DDRC |= (1<<FAN_PIN);
    *myTCCR1A = 0b10000001;
    *myTCCR1B = 0b00011001;
    *myICR1 = 40000;
    *myOCR1A = 0;
}

//==================adc drivers=================================

//=================Arduino====================
void setup()
{
    cli(); //global ints off
    U0init(9600);
    adc_init();
    fanInit();

    //LEDs as outputs
    *portDDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3);

    initStartButtonISR();
    sei(); //global ints on
    
    rtc.begin();
    dht.begin();
    LCD.begin(16,2);
    ventStepper.setSpeed(10);
    setState(DISABLED);

}


unsigned long t_fast=0, t_slow=0;
void loop(){

    if(g_btn){
        g_btn=false; 
        setState(g_state==DISABLED?IDLE:DISABLED); 
    }

    if(g_state!=DISABLED && millis()-t_fast>=FAST_MS){
        t_fast=millis();
        unsigned int w=adc_read(WATER_CH);
        unsigned char pct=w*100UL/1023UL;
        LCD.setCursor(0,1); 
        LCD.print("W:"); 
        LCD.print(pct); 
        LCD.print("%   ");
        if(w<WATER_THR){
            setState(ERROR);
        }

        unsigned int v=adc_read(VENT_CH);
        int steps=map((long)v,0,1023,-100,100);
        ventStepper.step(steps);
    }

    if(g_state!=DISABLED && millis()-t_slow>=SLOW_MS){
        t_slow=millis();
        float T=dht.readTemperature();
        float H=dht.readHumidity();
        LCD.setCursor(0,0);
        LCD.print("T:"); 
        LCD.print(T,1); 
        LCD.print((char)223); 
        LCD.print("C ");
        LCD.print("H:"); 
        LCD.print(H,0); 
        LCD.print("% ");

        if(g_state!=ERROR){
            if(g_state==IDLE && T>=TEMP_HI){
                *myOCR1A=*myICR1*0.6;
                setState(RUNNING); 
                U0printString(" FAN ON\n");
            }
            else if(g_state==RUNNING && T<=TEMP_LO){
                *myOCR1A=0;
                setState(IDLE);    
                U0printString(" FAN OFF\n");
            }
        }
    }
}
