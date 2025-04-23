//Monitor the water levels in a reservoir and print an alert when the level is too low
//Monitor and display the current air temp and humidity on an LCD screen.
//● Start and stop a fan motor as needed when the temperature falls out of a specifed
//range (high or low).
//● Allow a user to use a control to adjust the angle of an output vent from the system
//● Allow a user to enable or disable the system using an on/of button
//● Record the time and date every time the motor is turned on or of. This information
//should be transmitted to a host computer (over USB)

volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portB     = (unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (unsigned char *) 0x24;
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

//fan 
const uint8_t TEMPHHIGH = 80; 
const uint8_t  TEMPLOW = 75;

void fan_temp(float TempF){
  static bool fanON
  if(!fanON && TempF >= TEMPHIGH){
    
  }
  else if(fanON && TempF <= TEMPLOW){
    
  }
  
}
