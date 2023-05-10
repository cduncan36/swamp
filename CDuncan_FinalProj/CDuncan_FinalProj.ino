#include <LiquidCrystal.h>

// stepper motor setup
#include <Stepper.h>

const int stepsPerRevolution = 900;  
const int rolePerMinute = 15;

Stepper myStepper(stepsPerRevolution, 2, 4, 3, 5);

#define WRITE_HIGH_PA(pin_num)  *port_a |= (0x01 << pin_num);
#define WRITE_LOW_PA(pin_num)  *port_a &= ~(0x01 << pin_num);

#define RDA 0x80
#define TBE 0x20

// clock module setup
#include <Wire.h>
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;

// humidity sensor setup; using methods from elegoo example; these only make use of the humidifier library
// as the function was rewritten to make use of the onboard timer.
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 42;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// enable actuated by interrupt
bool errorFlag = 0;
bool measureFlag = 0;
bool enableFlag = 0;
volatile bool coolEnable = 0;
int waterThresh = 225;
int waterLevel;

float tempThresh = 28;
float temp;
float humidity;

// state check vars
// state table:
// INIT -1
// DISABLED 0
// IDLE 1
// RUNNING 2
// ERROR 3
int lastState = -1;
int newState = 0;

//global ticks counter
int currentTicks;
int timer_running;
int timerTally;

// port A registry pointers
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

// port B registry pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

// pin change interrupt pointers
volatile unsigned char* pcicr = (unsigned char*) 0x68;
volatile unsigned char* pcmsk0 = (unsigned char*) 0x6B;

// Timer Pointers
volatile unsigned char *myTCCR1A  = 0x80;
volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned char *myTCCR1C  = 0x82;
volatile unsigned char *myTIMSK1  = 0x6F;
volatile unsigned char *myTIFR1   = 0x36;
volatile unsigned int  *myTCNT1   = 0x84;

// UART Pointers
volatile unsigned char *myUCSR0A  = 0x00C0;
volatile unsigned char *myUCSR0B  = 0x00C1;
volatile unsigned char *myUCSR0C  = 0x00C2;
volatile unsigned int  *myUBRR0   = 0x00C4;
volatile unsigned char *myUDR0    = 0x00C6;

// ADC pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  // put your setup code here, to run once:

  // interrupt pins setup; PB0 PB1 = start stop buttons
  // enable pin change interrupt 0 (PCIE0) in PCICR; PCINT7:0 now calling interrupts
  *pcicr |= 0b00000001;
  // enable pins 0 1 in pc mask reg 0
  *pcmsk0 |= 0b00000011;
  // set PB0 PB1 to input
  *ddr_b &= 0b11111100;
  // enable pullup on PB0 PB1; PCINT0 PCINT1 pins
  *port_b |= 0b00000011;  

  // set PA0-6 to output; led lights 0-3, motor enable 4, motor direction 5-6
  *ddr_a |= 0b01111111;  
  // set PA7 to input; reset button
  *ddr_a &= 0b01111111;
  // PA7 pullup for reset button; drive PA5 high; PA6 remains low, motor direction
  *port_a |= 0b10100000;

  lcd.begin(16, 2);
  setup_timer_regs();
  adc_init();
  U0Init(9600);
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);
  myStepper.setSpeed(rolePerMinute);

  putStr("Startup complete");
  putChar('\n');
}

void loop() {
  // put your main code here, to run repeatedly:
  // universal enable check
  if (coolEnable) {
    // vent update
    ventUpdate();
    
    // if the timer is not already running, start it
    if(!timer_running)
    {
      // update currentticks to give clock cycle of 4s
      currentTicks = 62500;
      // start the timer, 1/1024 prescaler 
      *myTCCR1B |= 0b00000101;
      // set the running flag
      timer_running = 1;
    }
    // update water level via ADC input
    waterLevel = adc_read(1);
    // update temp/humidity thru module
    if (measure_environment(&temp, &humidity)) {
      // push temp/humidity to LCD
      lcd.clear();
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.print("C");
      lcd.setCursor(0,1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
    }

    // proceed to discriminate running state
    if (waterLevel >= waterThresh) {
      // water sufficient, non-error, reset error flag
      errorFlag = 0;
      // check temperature value to determine running/idle
      if (temp <= tempThresh) {
        // temperature sufficient, IDLE STATE
        newState = 1;
        // gLED on, all others off
        LEDset(1);
        // motor off
        motorSet(0);

      } else {
        // temperature insufficient, RUNNING STATE
        newState = 2;
        // bLED on, all others off
        LEDset(2);
        // motor on
        motorSet(1);
      }
    } else {
      // water insufficient, ERROR STATE
      newState = 3;
      // rLED on, all others off
      LEDset(3);
      // motor off
      motorSet(0);
      // error message to LCD
      lcd.clear();
      lcd.print("Water level is");
      lcd.setCursor(0,1);
      lcd.print("too low");
      // error message to log if error is new, trip error flag
      if(!errorFlag) {
        errorFlag = 1;
        statePrint();
        putStr("New state: ERROR");
        putChar('\n');
        putChar('\n');
      }
      // set currentticks to stop value
      currentTicks = 65535;
      // if the timer is running
      if(timer_running)
      {
        // stop the timer
        *myTCCR1B &= 0xF8;
        // set the flag to not running
        timer_running = 0;
        // reset tally
        timerTally = 0;
      }
      // reset measurement flag
      measureFlag = 0;
      // while-freeze till reset button low OR DISABLED
      while ((*pin_a & 0b10000000) && coolEnable) {
        // continue to update vent
        ventUpdate();
      }
      // update waterlevel on exit to avoid loops back to error state
      waterLevel = adc_read(1);
    }

  } else {
    // DISABLED STATE, fan off, yLED on, all others off, reset error and measurement flags
    newState = 0;
    // set currentticks to stop value
    currentTicks = 65535;
    // if the timer is running
    if(timer_running)
    {
      // stop the timer
      *myTCCR1B &= 0xF8;
      // set the flag to not running
      timer_running = 0;
      // reset tally
      timerTally = 0;
    }
    errorFlag = 0;
    measureFlag = 0;
    LEDset(0);
    motorSet(0);
  }

  // constant state check and var update; ignores error flag, which is handled separately prior to its freeze
  if ((newState != lastState) && (newState != 3)) {
    // log state change
    statePrint();
    switch(newState) {
      case 0:
        // DISABLED
        lcd.clear();
        lcd.print("Disabled");
        putStr("New state: DISABLED");
        break;
      case 1:
        // IDLE
        putStr("New state: IDLE");
        break;
      case 2:
        // RUNNING
        putStr("New state: RUNNING");
        break;
      case 3:
        // ERROR, no message 
        break;
    }
    putChar('\n');
    putChar('\n');
  }

  lastState = newState;  
}

// button interrupt ISR
ISR(PCINT0_vect) {
  // if PB0 low, start button pressed; enable program
  if ((*pin_b & 0b00000001) == 0) {
    coolEnable = 1;
  }
  // if PB1 low, stop button pressed; disable program
  if ((*pin_b & 0b00000010) == 0) {
    coolEnable = 0;
    motorSet(0);    
  }
}

// timer overflow ISR
ISR(TIMER1_OVF_vect)
{ 
  // Stop the Timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer, 1/1024 prescaler 
  *myTCCR1B |= 0b00000101;
  // if it's not the STOP amount, increment tally thru 15 4-sec cycle loop; else reset to 0
  if(currentTicks != 65535)
  {
    if (timerTally < 15) {
      timerTally += 1;
    } else {
      timerTally = 0;
      measureFlag = 0;
    }
  } else {
    timerTally = 0;
    measureFlag = 0;
  }
}

// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0x00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0b00000001;
}

// state table:
// DISABLED 0
// IDLE 1
// RUNNING 2
// ERROR 3
void LEDset(int state) {
  switch(state) {
    case 0:
      // DISABLED, yellow on
      WRITE_HIGH_PA(0);
      WRITE_LOW_PA(1);
      WRITE_LOW_PA(2);
      WRITE_LOW_PA(3);
      break;
    case 1:
      // IDLE, green on
      WRITE_HIGH_PA(1);
      WRITE_LOW_PA(0);
      WRITE_LOW_PA(2);
      WRITE_LOW_PA(3);
      break;
    case 2:
      // RUNNING, blue on
      WRITE_HIGH_PA(2);
      WRITE_LOW_PA(0);
      WRITE_LOW_PA(1);
      WRITE_LOW_PA(3);
      break;
    case 3:
      // ERROR, red on 
      WRITE_HIGH_PA(3);
      WRITE_LOW_PA(0);
      WRITE_LOW_PA(1);
      WRITE_LOW_PA(2);
      break;
  }
}

// sets motor running state; 0 off, 1 on
void motorSet(bool state) {
  if (state) {
    WRITE_HIGH_PA(4);
  } else {
    WRITE_LOW_PA(4);
  }
}

// global var to store previous vent state
int prevVState = 0;

// updates vent position and logs
void ventUpdate() {
  int stateIn = adc_read(0) / 100;
  if (stateIn != prevVState) {
    myStepper.step(100*(stateIn - prevVState));
    statePrint();
    putStr("New vent position: ");
    if (stateIn >= 10) {
      putChar('1');
      putChar((stateIn - 10) + '0');
    } else {
      putChar(stateIn + '0');
    }
    putChar('\n');
    putChar('\n');
  }
  prevVState = stateIn;
}

// measure_environment from elegoo example; rewritten to major extent to make use of onboard timer;
// onboard timer resets measureflag once every minute via interrupt, measureflag is tripped when a 
// valid measurement is registered. measureflag also resets on changes to disabled/error states, as
// to prompt a timely measurement upon reactivation
static bool measure_environment( float *temperature, float *humidity )
{
  // check if awaiting new measurement
  if(!measureFlag)
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      // trip new measurement flag
      measureFlag = 1;
      return( true );
    }
  }

  return( false );
}

// UART
void U0Init(int U0baud)
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
void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void putStr(unsigned char * inStr) {
  for(int i=0; i<strlen(inStr); i++) {
    putChar(inStr[i]);
  }
}

void int_print(unsigned int out_num)
{
  unsigned char print_flag = 0;  
  unsigned char lz_flag = 0;

  if(out_num >= 1000) {
    putChar(out_num / 1000 + '0');
    print_flag = 1;
    lz_flag = 1;
    out_num = out_num % 1000;    
  }
  if(out_num >= 100 || print_flag) {
    putChar(out_num / 100 + '0');
    print_flag = 1;
    lz_flag = 1;
    out_num = out_num % 100;    
  }
  if(out_num >= 10 || print_flag) {
    putChar(out_num / 10 + '0');
    print_flag = 1;
    lz_flag = 1;
    out_num = out_num % 10;    
  }
  if(!lz_flag) {
    putChar('0');
  }
  putChar(out_num + '0');
}

void statePrint() {
  putStr("Change detected at ");
  dt = clock.getDateTime();
  int_print((int)dt.month);
  putChar('/');
  int_print((int)dt.day);
  putChar('/');
  int_print((int)dt.year);
  putChar(' ');
  int_print((int)dt.hour);
  putChar(':');
  int_print((int)dt.minute);
  putChar(':');
  int_print((int)dt.second);
  putChar('\n');
}

// ADC
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  //*my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  //*my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference

  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference

  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
