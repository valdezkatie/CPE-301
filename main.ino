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


