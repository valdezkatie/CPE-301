//Monitor the water levels in a reservoir and print an alert when the level is too low
//Monitor and display the current air temp and humidity on an LCD screen.
//● Start and stop a fan motor as needed when the temperature falls out of a specifed
//range (high or low).
//● Allow a user to use a control to adjust the angle of an output vent from the system
//● Allow a user to enable or disable the system using an on/of button
//● Record the time and date every time the motor is turned on or of. This information
//should be transmitted to a host computer (over USB)

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

//===========LEDS========
#define LED_DIS PB1 //D52
#define LED_IDLE PB3 //D50
#define LED_RUN PB2 //D51 
#define LED_ERR PB0 //D53

//===========Btn=========
#define BTN_START 18
#define BTN_RESET 19
#define BTN_VENT_UP 30
#define BTN_VENT_DN 36 

//========temps=======
#define TEMP_THRESHOLD 26.0
#define WATER_THRESHOLD 300.0

//=============fan================
#define FAN_EN_BIT  PC2   //D35  EN
#define FAN_IN1_BIT PC0   //D37  IN1
#define FAN_IN2_BIT PC1   //D36  IN2

//===========stepper============
#define STEPPER_IN1 23
#define STEPPER_IN2 25
#define STEPPER_IN3 27
#define STEPPER_IN4 29
Stepper ventStepper(2038, STEPPER_IN1, STEPPER_IN3,STEPPER_IN2, STEPPER_IN4);  

//ADC
 #define WATER_CH 1 //PF1 / A1
 #define VENT_POT_CH 0 //PF0 / A0

///=======TIMING=======
//#define FAST_MS 500UL
//#define SLOW_MS 60000UL

//low level registers
#define RDA 0x80
#define TBE 0x20
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;

//this is from water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

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


//global objects
LiquidCrystal LCD(12,11,5,4,3,2);
DHT dht(45, DHT11);                 
RTC_DS1307 rtc;

//============states==========
enum CoolerState : uint8_t { DISABLED, IDLE, RUNNING, ERROR };
volatile CoolerState g_state = DISABLED;
volatile CoolerState g_next  = DISABLED;
volatile bool g_btn = false;
uint16_t waterLevel = 0;
unsigned long tFast = 0, tSlow = 0;

//===============UART=============
void U0init(int baud)
{
    uint16_t t = (16000000UL/16/baud)-1;
    *myUBRR0  = t;
    *myUCSR0A = 0x20;
    *myUCSR0B = 0x18;
    *myUCSR0C = 0x06;
}
void U0put(char c) { 
    while(!(*myUCSR0A & TBE)); 
    *myUDR0 = c; 
}

void U0print(const char* s) { 
    while(*s) U0put(*s++); 
}

 void logTime(){
    DateTime n = rtc.now();
    U0put('[');
    U0put('0' + n.hour()/10);   U0put('0' + n.hour()%10);   U0put(':');
    U0put('0' + n.minute()/10); U0put('0' + n.minute()%10); U0put(':');
    U0put('0' + n.second()/10); U0put('0' + n.second()%10); U0put(']');
 }

//==================adc drivers=================================
void adc_init()
{
    *my_ADCSRA = 0b10000111;
    *my_ADMUX  = 0b01000000;
}

unsigned int adc_read(unsigned char ch) 
{
    *my_ADMUX = (*my_ADMUX & 0xF8) | (ch & 0x07);
    (ch>7) ? (*my_ADCSRB |= 0x08) : (*my_ADCSRB &= ~0x08);
    *my_ADCSRA |= 0x40;
    while(*my_ADCSRA & 0x40);
    return *my_ADC_DATA;
}

//================LEDandFans==================================
 void setLEDs() {
     uint8_t mask = 0;
    if(g_state == DISABLED){
     mask |= (1<<LED_DIS);
    }
    else if(g_state == IDLE){
     mask |= (1<<LED_IDLE);
    }
    else if(g_state == RUNNING){
     mask |= (1<<LED_RUN);
    }else{
     mask |= (1<<LED_ERR); 
    }   
    PORTB = (PORTB & ~0x0F) | mask;
 }
void startFan() {
  DDRC |= (1 << PC0) | (1<< PC1) | (1<<PC2);

  //PORTE |= (1 << PC2);
  PORTC |= (1 << PC0);
  PORTC &= ~(1 << PC1);
  PORTC |= (1 << PC2);
 }
void stopFan() {
  DDRC |= (1 << PC0) | (1<< PC1) | (1<<PC2);

  //PORTE |= (1 << PC2);
  PORTC |= (1 << PC0);
  PORTC &= ~(1 << PC1);
  PORTC &= ~(1 << PC2);
 }

//================LEDandFans==================================
 void displayTempHum() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("T: ");
  LCD.print(t,1);
  LCD.print((uint8_t)223);
  LCD.print("C");
  LCD.setCursor(0, 1);
  LCD.print("H: ");
  LCD.print(h,0);
  LCD.print("%");
 }

//================buttons=====================================

 void startStopISR() { 
     g_btn = true; 
 }
 void resetISR() {
     if(g_state==ERROR && adc_read(WATER_CH) > WATER_THRESHOLD){
         g_next = IDLE;
     }       
 }
//================vent=====================================
 void controlVent() {
  if(!digitalRead(BTN_VENT_UP)){
   ventStepper.step(+100);
   U0print("Vent UP ");
   logTime();  
   U0put('\n');
   delay(150);
    }
    if(!digitalRead(BTN_VENT_DN)){
     ventStepper.step(-100);
     U0print("Vent DOWN "); 
     logTime();  
     U0put('\n');
     delay(150);
    }
 }

//=================Arduino====================
void setup()
{
    cli();
    U0init(9600);
    adc_init();
    DDRB |= 0x0F;
    DDRC |= (1<<FAN_EN_BIT)|(1<<FAN_IN1_BIT)|(1<<FAN_IN2_BIT);
    LCD.begin(16,2);
    dht.begin();
    rtc.begin();
    ventStepper.setSpeed(10);
    pinMode(BTN_VENT_UP, INPUT_PULLUP);
    pinMode(BTN_VENT_DN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_START), startStopISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BTN_RESET), resetISR, RISING);
    sei();
    setLEDs();
}

 void loop() {
     if(g_btn){ 
         g_btn = false; 
         g_next = (g_state==DISABLED) ? IDLE : DISABLED; 
     }
 
     if(millis()-tFast >= 500){
         tFast = millis();
         waterLevel = adc_read(WATER_CH);
         controlVent();
     }
     if(millis()-tSlow >= 60000){
         tSlow = millis();
         displayTempHum();
     }
 
     float temp = dht.readTemperature();
     switch(g_state){
         case DISABLED: stopFan(); 
            if(g_next==IDLE) displayTempHum(); 
            break;
         case IDLE:
             stopFan();
             if(waterLevel <= WATER_THRESHOLD){
                g_next = ERROR;
             }
             else if(temp > TEMP_THRESHOLD) {
                g_next = RUNNING;
             }
             break;
         case RUNNING:
             startFan();
             if(waterLevel <= WATER_THRESHOLD) {
                 g_next = ERROR;
             }
             else if(temp <= TEMP_THRESHOLD) {
                 g_next = IDLE;
             }
             break;
         case ERROR: stopFan(); 
             break;
     }
 
     if(g_state != g_next){
         U0print("State "); U0put('0'+g_state); U0print(" -> "); U0put('0'+g_next); U0put(' ');
         logTime(); U0put('\n');
         g_state = g_next;
         setLEDs();
     }
     delay(50);
 }
