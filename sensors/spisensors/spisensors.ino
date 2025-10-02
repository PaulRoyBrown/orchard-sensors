
// Handle two BME280 sensors to take measurements by SPI

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <limits.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RCSwitch.h>


//#define SHOW_SERIAL  // Define to trace on serial port
//#define NO_SENSOR    // For bit math tests 

#define MAX_AVG_SAMPLES 5

#define RF_POWER_PIN 9
#define RF_TX_PIN 4
#define SIZE_BITS 32 //20

RCSwitch mySwitch = RCSwitch();

// Pins for the two SPI sensors. Same SCK/MISO/MOSI but two different pind for CS
// We have a SPI BME280 sensor attached to a socket in the board, and a second one in a cable end 
// So we need 4 cables for SPI, and two more for GND/Vcc=>6 cables 
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS_BOARD 10
#define BME_CS_CABLE 8

Adafruit_BME280 bmeBoard(BME_CS_BOARD); // hardware SPI
Adafruit_BME280 bmeCable(BME_CS_CABLE); // hardware SPI

// flags to store if BME280 is detected
bool boardOk = false; 
bool cableOk = false;

#define LED_STATE 2

#define SEALEVELPRESSURE_HPA (1013.25)

// WDT entry count. Wakeup each 8 secs, and each 8 times we wakeup do measuremnts
#define MAX_AWAKES 8
#define MAX_AWAKES_RESET 1000 // Reset each 8000secs

volatile uint8_t awakes = MAX_AWAKES; // 8, so first time in loop,we send information to make checks easier (do not wait 60 secs) after power on
volatile uint8_t maxAwakes = MAX_AWAKES; // This is increased if low voltage to save batery
volatile uint8_t reset = 0; 

// Power save varaibles
#define VOLTAGE_HIGH    4100//4061
#define VOLTAGE_MEDIUM  4060 //   4061//4050//4010
#define VOLTAGE_LOW     4030 //4050 //4000//3959
#define VOLTAGE_ULTRALOW  3990 //3995 // 3770//3950

#define POWER_STATE_HIGH    1  // 1*8*MAX_AWAKES sec -> aprox 8*8 = 1 min
#define POWER_STATE_MEDIUM  3  // 3*8*MAX_AWAKES sec -> aprox 3*8*8 = 3 min aprox
#define POWER_STATE_LOW     10 // 10*8*MAX_AWAKES sec -> aprox 10*8*8 = 10 min aprox
#define POWER_STATE_ULTRALOW   30 // 30*8*MAX_AWAKES sec -> aprox 30*8*8 = 30 min aprox

uint8_t powerState = POWER_STATE_HIGH ; // ULTRALOW,LOW,MEDIUM,HIGH Actual power state
uint8_t powerSaveLevel = 0;
bool bReboot = false;

// Sensor data stored here
typedef struct _SensorData
{
  int8_t temp;
  int8_t humidity;
  int8_t pressure;
}
SensorData;

SensorData getValues(Adafruit_BME280 &);

unsigned long delayTime = 100;

void(* resetFunc) (void) = 0;

//////////////////////////////////////////////////////////////////////////
// WDT Interrupt
// Our WDT triggers this interrupt periodically. We increase counters so we can take measurement when 
// MAX_AWAKES happen
ISR(WDT_vect)
{
  awakes++;
  reset++;
}

// Function Pototype
void wdt_init(void) __attribute__ ((naked, used, section(".init3")));

// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
    return;
}

//////////////////////////////////////////////////////////////////////////
// Sleep Configuration Function
//   Also wake-up after

void enterSleep(void)
{  
  WDTCSR |= _BV(WDIE); // Enable the WatchDog before we initiate sleep

  set_sleep_mode(SLEEP_MODE_PWR_SAVE);    /* Some power Saving */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    /* Even more Power Savings */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();
  sleep_bod_disable();  // Additionally disable the Brown out detector

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}

// Measure battery voltage
long readVcc() 
{
  // With this method, comsumption gos from 90uA to 245uA
  //return 0;
  uint8_t copy =   (*(volatile uint8_t *)(0x7C));
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(4); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  // read it a second time
 ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = 0L;

  result = (high<<8) | low;
  //calibrate the Arduino by changing the number below so that the reading agrees with your multimeter
  result = /*1125300L*/ /*1121208*/ /*1343398*/ /*1385122*/ /*1349475*/ 1408677 / result; // Calculate Vcc (in mV); 1125300 = 1.1*1024*1000 (2692/1125300 = 3220/x ->x = 3220*1125300/2692 = 4.01/4.790 = x/1346012 x = 
  
  ADMUX = copy;
/*
1385122        x 
-------  = ---------    => x = 1385122* (5.3/5.44) = 1349475
5.544        5.30 

1349475       x
------- = ---------       => x = 1408677
 3.989        4.164
 */
 return result; // Vcc in millivolts
}

// Methods to measure battery voltage
unsigned long samples[MAX_AVG_SAMPLES];
unsigned long  mtotal = 0;
unsigned long num_samples = 0;

unsigned long add_voltage_sample(unsigned long sample)
{
  unsigned long avgVoltage = 0;

    mtotal += sample;
    if (num_samples < MAX_AVG_SAMPLES)
        samples[num_samples++] = sample;
    else
    {
        unsigned long& oldest = samples[num_samples++ % MAX_AVG_SAMPLES];
        mtotal -= oldest;
        oldest = sample;

        avgVoltage = mtotal / MAX_AVG_SAMPLES;
    }

    return avgVoltage;
}

/* voltage in power output / voltage measured in arduino
 * 4.58 -> 4632
 * 4.40   4411
 * 4.3  4275
 * 4.2 4184
 * 4.1 4061 -> HIGH  (a 100% loaded batery gives 4.12V)
 * 4.0 4050 -> MEDIUM
 * 3.9 4050
 * 3.8 4050
 * 3.7 4050
 * 3.6 4014 -> LOW 
 * 3.5 3901 -> 3.25 (3904 se queda en 3.15V,medido en los pines de arduino, al transmitir)
 * 3.4 3763
 * 3.3 3654 
 * 3.2 3524
 */
// 1343398/3427 = x/3180 -> x=(1343398/3427)*3180 = 1246572
// 1343398/3928 = x/4050

uint8_t get_power_state(unsigned long voltage)
{
  uint8_t state = 0;
  const char *str = "";

  if(/*voltage == 0 ||*/ voltage >= VOLTAGE_HIGH)
  {
    state = POWER_STATE_HIGH; // 1 update per minute aprox
    if(powerState != state)
        str = "ENTER POWER STATE HIGH";
    powerState = state;
    powerSaveLevel = 0;
  }
  else if(voltage < VOLTAGE_HIGH && voltage >= VOLTAGE_MEDIUM)
  {
    state = POWER_STATE_MEDIUM; // 1 update each 3 min aprox
    if(powerState != state)
        str = "ENTER POWER STATE MEDIUM";
    powerState = state;
    powerSaveLevel = 1;
  }
  else if(voltage < VOLTAGE_MEDIUM && voltage >= VOLTAGE_LOW)
  {
    state = POWER_STATE_LOW; // 1 update each 10min aprox
    if(powerState != state)
        str = "ENTER POWER STATE LOW";
    powerState = state;
    powerSaveLevel = 2;
  }  
  else
  {
    state = POWER_STATE_ULTRALOW; // 1 update each 30min aprox
    if(powerState != state)
        str = "ENTER POWER STATE ULTRA LOW";
    powerState = state;
    powerSaveLevel = 3;
  }

#ifdef SHOW_SERIAL
  Serial.println(str);
#endif

  return state;
}

void setup_wdt()
{
   /*** Setup the WDT ***/
    cli();
    /* Clear the reset flag. */
    MCUSR &= ~(1<<WDRF);

    /* In order to change WDE or the prescaler, we need to
    * set WDCE (This will allow updates for 4 clock cycles).
    */
    WDTCSR |= (1<<WDCE) | (1<<WDE);

    /* set new watchdog timeout prescaler value */
    //WDTCSR = 1<<WDP1 | 1<<WDP2;             /* 1.0 seconds */
    //WDTCSR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2; /* 2.0 seconds */
    //WDTCSR = 1<<WDP3;                     /* 4.0 seconds */
    WDTCSR = 1<<WDP0 | 1<<WDP3;           /* 8.0 seconds */

    /* Enable the WD interrupt (note no reset). */
    //WDTCSR |= _BV(WDIE); // Not here but when we go to Sleep
    sei();
}

////////////////////////////////
// Methods to pack in 32bit the measureents
// We will have two types of messages identified by 4bit headers
int8_t set_val(int8_t x,bool sig)
{
  int8_t sign=0;
  char val =0x0;
  
  if(sig)
  {
    // Use first bit for sign
    if(x>=0)
    {
      sign = 0;
      val=  (x & 0x7F);
    }
    else
    {
      sign = 1;
      val = (sign << 6) |(abs(x) & 0x7F);
    }
  }
  else
  {
      // 7 bits used for value
      val = (x & 0x7F);      
  }

  return val;
}

/*
|head | temp1  |temp2 |hum1 | hum2  
 1010 | 7 bit |7 bit |7 bit|7 bit

|head |voltage | press| state | reserved
|1011 |  13bit | 7bit | 2bit  | 000000
*/
unsigned long set_message(uint8_t head,int8_t t1,int8_t t2,int8_t h1,int8_t h2)
{
  unsigned long val = 0;  
  
  uint16_t x1 = ((uint16_t)set_val(h1,false) << 7 ) | ((uint16_t)set_val(h2,false));
  uint16_t x2 = ((uint16_t)set_val(t1,true) << 7 ) | ((uint16_t)set_val(t2,true) );

  val =  ( ( (uint32_t)head << 28) |((uint32_t)x2 << 14)| ((uint32_t) (x1 & 0x3FFF)) );  

    // head  16        -1     2       127
  //  1010 0010000 1000001 0000010 1111111
  //  val=   10000 1000001 0000010 1111111
#ifdef SHOW_SERIAL
  Serial .print("val=");  
  Serial.print(val,BIN);
  Serial .println("");
#endif

  return val; 
}
/*
head |voltage | press| state | reserved
|1011 |  13bit | 7bit | 2bit  | 000
pressure has sign because we have substracted 1000 and could be negative
*/
unsigned long set_message2(uint8_t head,unsigned long volt,int8_t pres,int8_t state)
{
  unsigned long val = 0;  
  
  uint16_t x1 = ((uint16_t)set_val(state,false) << 3 ) | (0x0) ;
  uint32_t x2 = ((uint32_t)volt << 7) | ((uint16_t)set_val(pres,true) ); // Pressure relative to 1000

  val =  ( ( (uint32_t)head << 28) |((uint32_t)x2 << 5)| ((uint32_t) (x1 & 0x1F)) ); 
#ifdef SHOW_SERIAL
  Serial .print("val=");  
  Serial.print(val,BIN);
  Serial .println("");
#endif
  
  return val;
}

void BME280_Sleep(uint8_t addr) 
{
  /*
  Wire.beginTransmission(addr);    // or 0x77
  Wire.write((uint8_t)0xF4);       // Select Control Measurement Register
  Wire.write((uint8_t)0b00000000); // Send '00' for Sleep mode
  Wire.endTransmission();
  */
    const uint8_t CTRL_MEAS_REG = 0xF4;
  // BME280 Register 0xF4 (control measurement register) sets the device mode, specifically bits 1,0
  // The bit positions are called 'mode[1:0]'. See datasheet Table 25 and Paragraph 3.3 for more detail.
  // Mode[1:0]  Mode
  //    00      'Sleep'  mode
  //  01 / 10   'Forced' mode, use either '01' or '10'
  //    11      'Normal' mode
  //Serial.println("BME280 to Sleep mode...");
  Wire.beginTransmission(addr);
  Wire.requestFrom(addr, 1);
  uint8_t value = Wire.read();
  value = (value & 0xFC) + 0x00;         // Clear bits 1 and 0
  Wire.write((uint8_t)CTRL_MEAS_REG);    // Select Control Measurement Register
  Wire.write((uint8_t)value);            // Send 'XXXXXX00' for Sleep mode
  Wire.endTransmission();
}

bool setupBoard(Adafruit_BME280 *board, bool bPressure, uint8_t addr)
{
  int n = 3;
  
    // Maybe we should clean Wire here,or use other one Wire object,but Wire is an extern
    // declared somewhere
    while( n>0 && !board->begin(0x77, &Wire)) 
    {
      delay(30);
      n--;
    }

    if(n == 0)
    {
#ifdef SHOW_SERIAL      
      Serial.println("Could not find a valid BME280 sensor, check sensor board wiring!");
#endif      
      return false;
    }

    board->setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    bPressure ? Adafruit_BME280::SAMPLING_X1 : Adafruit_BME280::SAMPLING_NONE, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF , Adafruit_BME280::standby_duration::STANDBY_MS_1000  );

    return true;
}

void setup() 
{
    delay(2000); // in case of continous crashes,try not to deplete battery  
    
    awakes = MAX_AWAKES;
    maxAwakes = MAX_AWAKES; 
    reset = 0; 
    powerState = POWER_STATE_HIGH ;
    powerSaveLevel = 0;
    bReboot = false;

    Serial.begin(9600);
    Serial.println("sensor_huerto_sleep"); 

    Serial.println("Initializing RF_TX"); 
    pinMode(LED_STATE, OUTPUT);

    mySwitch.enableTransmit(RF_TX_PIN);
    mySwitch.setRepeatTransmit(20);
     
   

#ifndef NO_SENSOR    
    Serial.println("BME280 initializing sensor board");
   
   boardOk = setupBoard(&bmeBoard,true,0x76);
    if(!boardOk)
    {
        Serial.println("Error initializing sensor board");
    }
 
    Serial.println("BME280 initializing sensor cable");
    cableOk = setupBoard(&bmeCable,false, 0x77);
    if(!cableOk)
    {
        Serial.println("Error initializing sensor cable");
    }
   
   // Serial.println("----------------------------------------");
  
    Serial.println("normal mode, 2x oversampling for all, filter off,");
    Serial.println("1000ms standby period");
    
    Serial.println();
#endif

  // turn off I2C
  TWCR &= ~(bit(TWEN) | bit(TWIE) | bit(TWEA));

    setup_wdt();

    delay(500);
}

void loop() {
     unsigned long voltage = 0;

    SensorData dat1 ={0,0};
    SensorData dat2 ={0,0};

  //Serial.println(awakes);
    if(awakes < maxAwakes)
    {
      if(reset>MAX_AWAKES_RESET)
      {      
        reset = 0;
        resetFunc();
      }

      if(reset == (MAX_AWAKES_RESET - 1))
      {
        // Why not when equal?
        // Because we want to see the efect in the grapth before reboot
        bReboot = true;        
      }
    
      //Serial.println(awakes);
      // This makes faster to detect if avg voltage has changed,instead of waiting to the whole number of wake ups it can continue
      // in next try outside this if,because the average is already calculated
      voltage = readVcc();
      add_voltage_sample(voltage);

      //Serial.println(voltage);

      /* Re-enter sleep mode. */
      enterSleep();
    
      return;
    }
    
    awakes = 0;
      
    // Only needed in forced mode! In normal mode, you can remove the next line.
    //bme.takeForcedMeasurement(); // has no effect in normal mode
  
    delay(delayTime); 
    
    
    //delay(5000);
 #ifdef SHOW_SERIAL
    Serial.print("Sensor Board: ");
 #endif   

    // Output pins for CS
    pinMode(BME_CS_BOARD, OUTPUT);
    pinMode(BME_CS_CABLE, OUTPUT);
    delay(delayTime); 
    
    digitalWrite(BME_CS_BOARD, LOW);
    digitalWrite(BME_CS_CABLE, HIGH);
    delay(delayTime); 
  
    //boardOk = setupBoard(&bmeBoard,false, 0x77);
#ifndef NO_SENSOR
    boardOk = setupBoard(&bmeBoard,true,0x76);
    if(boardOk)
    {
      bmeBoard.takeForcedMeasurement();
      dat1 = getValues(bmeBoard);
    }
    else
    {
      // Mark now with special value
      dat1.temp = -1;
      dat1.humidity = 1;

      delay(10000);
      // Retry
      boardOk = setupBoard(&bmeBoard,true, 0x76);
    }
#else
  dat1.temp = 16;
  dat1.humidity = 2;   
#endif

   delay(delayTime);
    
    /*************************************************/
#ifdef SHOW_SERIAL   
    Serial.print("Sensor Cable: ");
#endif   
    digitalWrite(BME_CS_BOARD, HIGH);
    digitalWrite(BME_CS_CABLE, LOW);
    
    delay(delayTime); 

#ifndef NO_SENSOR
    cableOk = setupBoard(&bmeCable,true,0x77);
    if(cableOk)
    {
      bmeCable.takeForcedMeasurement();
      dat2 = getValues(bmeCable);
    }
    else
    {
      dat2.temp = -2;
      dat2.humidity = 2;

      delay(10000);
      cableOk = setupBoard(&bmeCable,false,0x77);  
      delay(delayTime);
    }
#else
    dat2.temp = 0;
    dat2.humidity = 127;
#endif
    
    // 1st message: temp & humidity
    unsigned long data = set_message(0xA,dat1.temp,dat2.temp,dat1.humidity,dat2.humidity);
    pinMode(RF_POWER_PIN, OUTPUT);
    digitalWrite(RF_POWER_PIN, HIGH);
    mySwitch.send(data, SIZE_BITS);
    
#ifdef SHOW_SERIAL 
    //show_bits_long(data);
    Serial.print("ValueSent: ");
    Serial.println(data);
#endif

    voltage = readVcc();
    unsigned long avgVoltage = add_voltage_sample(voltage);

    // No samples enought for average yet
    if(avgVoltage<=0)
      avgVoltage = voltage;

    if(avgVoltage > 0)
    {
      uint8_t state = get_power_state(avgVoltage);
      maxAwakes = state * MAX_AWAKES;
    }

  uint8_t level = powerSaveLevel;
  // Marker in graphs if we just come from a reboot
  if(bReboot)
  {
    level = 4;  // a momentary power state=4 should appear in graph just before reboot
    bReboot = false;    
  }

  // 2nd message: battery level and pressure
  unsigned long message = set_message2(0xB,avgVoltage,dat2.pressure,level);

  level = 0;
  
#ifdef SHOW_SERIAL   
    Serial.print("Vcc is ");
    Serial.print(avgVoltage>0?avgVoltage:voltage);
    Serial.print(" Sent:");
     Serial.print(message);
    Serial.print(" mV state=");
    Serial.println(powerState);
    Serial.print("\n");
#endif
    
    //delay(delayTime);
    
    // Guard. We use it to send the state.No matter if this transmit is lost
    
    mySwitch.send(message, SIZE_BITS);
    
    // Try putting pins as input to try saving power
    pinMode(RF_POWER_PIN, INPUT);
    digitalWrite(RF_POWER_PIN, LOW);
 
    digitalWrite(BME_CS_BOARD, LOW);
    digitalWrite(BME_CS_CABLE, LOW);

    delay(delayTime); 
   
    pinMode(BME_CS_BOARD, INPUT);
    pinMode(BME_CS_CABLE, INPUT);

    //BME280_Sleep(0x76); does it work?
    //BME280_Sleep(0x77);

    /* Re-enter sleep mode. */
    enterSleep();
}


SensorData getValues(Adafruit_BME280 &bme) 
{
    SensorData dat{0,0};

#ifdef NO_SENSOR
  return dat;  
#else
    float temp = bme.readTemperature();

#ifdef SHOW_SERIAL      
    //Serial.print("Temperature = ");    
    Serial.print(temp);
    Serial.print(" *C");
    Serial.print(" \t\t"); 
#endif
    
    float pressure = bme.readPressure() / 100.0F;

#ifdef SHOW_SERIAL   
    //Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.print(" hPa");
    Serial.print("\t\t"); 
#endif
    
    //Serial.print("Approx. Altitude = ");
    //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    //Serial.println(" m");

    float humidity = bme.readHumidity();

 #ifdef SHOW_SERIAL   
    //Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.print(" %");
    Serial.println("\t\t"); 
#endif

    dat.temp = (uint8_t)temp; //trunc(temp*100)/100;
    dat.humidity = (uint8_t)humidity;
    dat.pressure = (pressure - 1000);
  
    return dat;
#endif
}

