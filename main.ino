//Monitor the water levels in a reservoir and print an alert when the level is too low
//Monitor and display the current air temp and humidity on an LCD screen.
//● Start and stop a fan motor as needed when the temperature falls out of a specifed
//range (high or low).
//● Allow a user to use a control to adjust the angle of an output vent from the system
//● Allow a user to enable or disable the system using an on/of button
//● Record the time and date every time the motor is turned on or of. This information
//should be transmitted to a host computer (over USB)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

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
volatile unsigned char *myTIMSK1  =(unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   =(unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   =(unsigned int *) 0x84;
//this is from water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//lcd pin set up
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal LCD(RS, EN, D4, D5, D6, D7);

//================4states======================================
enum CoolerState : uint8_t { DISABLED, IDLE, RUNNING, ERROR };

//global state default disabled and global start button
volatile CoolerState g_state = DISABLED;
volatile bool g_startBtn = false;



//setState() turns on one LED and prints a letter
void setState(CoolerState s)
{
    g_state = s;

    /* LED mux: clear PB1-PB4, then set one bit */
    *portB &= ~0b00011110; // PB1–PB4 low
    switch (s) {
        case DISABLED: *portB |= (1<<PB1); break;  // yellow
        case IDLE: *portB |= (1<<PB2); break;  // green
        case RUNNING: *portB |= (1<<PB3); break;  // blue
        case ERROR: *portB |= (1<<PB4); break;  // red
    }

    //minimal serial feedback
    while (!(*myUCSR0A & TBE));
    *myUDR0 = "DIRE"[s];
}

//================4states=====================================

//================button=====================================

void initStartButtonISR() {
  volatile unsigned char *portD = (unsigned char *)0x2B;
  volatile unsigned char *portDDRD = (unsigned char *)0x2A;

  *portDDRD &= ~(1<<DDD2); //PD2 = input
  *portD |= (1<<PORTD2); //enable pull-up (idle HIGH)
  
  EIMSK |= (1<<INT0); //unmask external interrupt 0
  EICRA |= (1<<ISC01); //trigger on falling edge
  //INT0 is on digital pin 2—wire button there
}

ISR(INT0_vect) {
  g_startBtn = true;
}
//================button=====================================

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

//==================adc drivers=================================
void adc_init() //write your code after each commented line and follow the instruction 
{
  // setup the A register
 // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0b10000000;
 // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b10111111;
 // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11011111;
 // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRB &= 0b11111000;
  // setup the B register
// clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;
 // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;
  // setup the MUX Register
 // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;
// set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
 // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;

}

unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
 
  *my_ADMUX &= 0b11100000;

  
  *my_ADCSRB &= 0b11110111;
 
  // set the channel selection bits for channel 0
 

  
  *my_ADCSRA |= 0x40;
  
  while((*my_ADCSRA & 0x40) != 0);
  
  
}
//==================adc drivers=================================

//=======================================
/*
void waterlevel(){
  if(water_level>=100){
    lcd.setCursor(0, 1);
    lcd.print("High");
    noTone(buzzer);
}
  else if(water_level>=50){
    lcd.setCursor(0, 1);
    lcd.print("Water level is too low");
    tone(buzzer, 1000,5);
} 


void tempandhumidity(){
  Serial.println();

 int chk = DHT11.read(DHT11PIN);

 Serial.print("Humidity (%): ");
 Serial.println((float)DHT11.humidity, 2);

  Serial.print("Temperature  (C): ");
  Serial.println((float)DHT11.temperature, 2);

  delay(2000);

}

//fan 
const uint8_t TEMPHHIGH = 80; 
const uint8_t  TEMPLOW = 75;

void fan_temp(float TempF){
  static bool fanON;
  if(!fanON && TempF >= TEMPHIGH){
    
  }
  else if(fanON && TempF <= TEMPLOW){
    
  }
  
}
*/

//============================================

//=================Arduino====================
void setup()
{
    cli(); //global ints off
    U0init(9600);

    //LEDs as outputs
    *portDDRB |= (1<<PB1)|(1<<PB2)|(1<<PB3)|(1<<PB4);

    initStartButtonISR();
    sei(); //global ints on

    setState(DISABLED);//start in disabled mode
}


void loop() {
  //if user pressed start/stop:
  if (g_startBtn) {
    g_startBtn = false;
    //toggle between DISABLED and IDLE
    if (g_state == DISABLED) {
      setState(IDLE);
    } else {
      setState(DISABLED);
    }
  }

  //add per-state logic later
}
//=================Arduino============================
