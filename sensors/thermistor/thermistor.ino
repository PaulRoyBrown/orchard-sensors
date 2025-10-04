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

#include <Adafruit_ADS1X15.h>
#include <math.h>


#define SHOW_SERIAL  // Define to trace on serial port
//#define NO_SENSOR    // For bit math tests 

#define MAX_AVG_SAMPLES 5

/////////////////////////////
// RCSwitch to transmit data
/////////////////////////////

#define RF_POWER_PIN 9
#define RF_TX_PIN 4
#define SIZE_BITS 32 
//#define LED_STATE 2  //ligh LED on transmit 

RCSwitch mySwitch = RCSwitch();

////////////////////////////
// One SPI BME280 sensor
// Standard SPI pins on Mini Pro
////////////////////////////

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS_BOARD 10

Adafruit_BME280 bmeBoard(BME_CS_BOARD); // hardware SPI
bool boardOk = false;

///////////////////////////
// For second sensor, we use a thermistor feeded with a constant current LM334 
// setted for THERMISTOR_CURRENT_UA microamps
///////////////////////////

#define THERMISTOR_POWER_PIN    8
#define THERMISTOR_CURRENT_UA   30.89 // Constant current µA. Important to be well measured!!
#define THERMISTOR_CURRENT_TIME 100
#define ADC_POWERED_BY_PIN // A try to see if I can reduce power comsuption by switching ADC with a GPIO

// We measure the thermistor voltage with this ADC chip
Adafruit_ADS1115 ads; //Global object

const float CURRENT_A = THERMISTOR_CURRENT_UA / 1e6; // Convert to amps
float multiplier = 0.0078125F; // Scale factor ADC

// Storage type for sensor data
typedef struct _SensorData
{
  int8_t temp;
  int8_t humidity;
  int8_t pressure;
  int8_t tempf; // decimal part
}
SensorData;

//////////////////////////
// Power state management
//////////////////////////
// WDT entry count. We wakeup each 2 o 8 secs (as configured in setup_wdt() just to check battery 
// voltage and each MAX_AWAKES we do measurements
#define MAX_AWAKES 8
#define MAX_AWAKES_RESET 1000 // Reset each 8000secs

volatile uint8_t awakes = MAX_AWAKES; // 8, so first time in loop,we send information to make checks easier (do not wait 60 secs) after power on
volatile uint8_t maxAwakes = MAX_AWAKES; // This is increased if low voltage to save batery
volatile uint8_t reset = 0; 

// Battery limits to start saving power
#define VOLTAGE_HIGH    4099
#define VOLTAGE_MEDIUM  4061//4050//4010
#define VOLTAGE_LOW     4050 //4000//3959
#define VOLTAGE_ULTRALOW  3995 // 3770//3950

// States that match above battery limits
#define POWER_STATE_HIGH    1  // 1*8*MAX_AWAKES sec -> aprox 8*8 = 1 min
#define POWER_STATE_MEDIUM  3  // 3*8*MAX_AWAKES sec -> aprox 3*8*8 = 3 min aprox
#define POWER_STATE_LOW     10 // 10*8*MAX_AWAKES sec -> aprox 10*8*8 = 10 min aprox
#define POWER_STATE_ULTRALOW   30 // 30*8*MAX_AWAKES sec -> aprox 30*8*8 = 30 min aprox

uint8_t powerState = POWER_STATE_HIGH ; // ULTRALOW,LOW,MEDIUM,HIGH Actual power state
uint8_t powerSaveLevel = 0;
bool bReboot = false;

SensorData getValues(Adafruit_BME280 &);

unsigned long delayTime = 100;

// Used to reset (i.e , when called it makes processor to crash and that's all...)
void(* resetFunc) (void) = 0;

///////////////////////////////////
// WDT Interrupt and related
//////////////////////////////////
ISR(WDT_vect)
{
  // Count times we wakeup
  awakes++;
  reset++;
}

///////////////////////////////////////
//  Enter sleep Configuration Function
//   Also wake-up after
//////////////////////////////////////
void enterSleep(void)
{ 
  WDTCSR |= _BV(WDIE); // Assure WDT active

  // Disable peripherals for power saving
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable(); // Close TWI without closing Wire

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode(); // Enter sleep here

  // After WDT wakeup, it continues here
  sleep_disable();
  power_all_enable();

  delay(50); // Wait I²C stabilization
}

///////////////////////////////////////////
// Methods for measuring battery voltage
///////////////////////////////////////////

// For averaging voltage battery measurement
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

// Use "internal ADC" to measure battery voltage
// (Check https://forum.arduino.cc/t/measure-3-7-battery-voltage/1033649/22)
// With this method, comsumption go from 90uA to 245uA !
long readVcc() 
{
  return (VOLTAGE_HIGH + 1); // Return this to stay in normal state
 
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

  delay(10); // Wait for Vref to settle
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
 * 3.5 3901 -> 3.25 (3904 goes to 3.15V when transmit)
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
  cli();           // Set no interrupst in this block
  
  wdt_reset();     // Reset WDT

  // Try to collect all config operations here.inly 4 clock intervals!
  // Set configuration mode
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Configure: 2 sec + interrupt, not reset
  WDTCSR = (1 << WDIE) | (1 << WDP0) | (1 << WDP1) | (1 << WDP2);

   // Configure: 8 sec + interrupt, not reset
  //WDTCSR = (1 << WDIE) | 1<<WDP0 | 1<<WDP3; 
  
  sei();           // Reset interrupts handling
}

// Pack value in 7 bit. 1st could be the sign if ig=true and six more the value
// So we can send temperatures from -64 to 64 (2⁶6)
// Altough we return 8bit here, in fact information is packed in the lower 7 bit
// Last bit will be ignored and proper shotfs and AND will be used
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

unsigned long set_message(uint8_t head,int8_t t1,int8_t t2,int8_t h1,int8_t h2)
{
  unsigned long val = 0;  
  
  uint16_t x1 = ((uint16_t)set_val(h1,false) << 7 ) | ((uint16_t)set_val(h2,false));
  uint16_t x2 = ((uint16_t)set_val(t1,true) << 7 ) | ((uint16_t)set_val(t2,true) );

  // head of 4bit starts at 28, then 14 bit for temperature as two 6bit values both with signn (so 14bit), and then 7+7=14bits for humidity (0 to 100)
  // That makes 32bit
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


unsigned long set_message3(uint8_t head,int8_t t1,int8_t t2,int8_t h1,int8_t t2f)
{
  unsigned long val = 0;  
  
  uint16_t x1 = ((uint16_t)set_val(h1,false) << 7 ) | ((uint16_t)set_val(t2f,false));
  uint16_t x2 = ((uint16_t)set_val(t1,true) << 7 ) | ((uint16_t)set_val(t2,true) );

  // head of 4bit starts at 28, then 14 bit for temperature as two 6bit values both with signn (so 14bit), and then 7+7=14bits for humidity (0 to 100)
  // That makes 32bit
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

void BME280_Sleep(uint8_t addr) 
{
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

/*
    digitalWrite(LED_STATE, HIGH); 
    delay(200);
    digitalWrite(LED_STATE, LOW); 
    delay(200);
*/
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

/////////////////////////////////////////
// Spline adjustemt of thermistor curve
////////////////////////////////////////

// Calibrated table of temp/resistance thermistor curve
// Values between -5 and 45 temperature degrees
const int N = 11;
const float temps[N] = {-5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45};
const float resist[N] = {12300, 9423, 7282, 5672, 4450, 3515, 2796, 2238, 1802, 1459, 1188};

float a[N], b[N], c[N], d[N], h[N - 1], alpha[N - 1], l[N], mu[N], z[N];

// IA generated for best fit 10 points to the curve
void computeSplineCoefficients() 
{
  for (int i = 0; i < N - 1; i++) {
    h[i] = temps[i + 1] - temps[i];
  }

  for (int i = 1; i < N - 1; i++) {
    alpha[i] = (3 / h[i]) * (resist[i + 1] - resist[i]) - (3 / h[i - 1]) * (resist[i] - resist[i - 1]);
  }

  l[0] = 1;
  mu[0] = 0;
  z[0] = 0;

  for (int i = 1; i < N - 1; i++) {
    l[i] = 2 * (temps[i + 1] - temps[i - 1]) - h[i - 1] * mu[i - 1];
    mu[i] = h[i] / l[i];
    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }

  l[N - 1] = 1;
  z[N - 1] = 0;
  c[N - 1] = 0;

  for (int j = N - 2; j >= 0; j--) {
    c[j] = z[j] - mu[j] * c[j + 1];
    b[j] = (resist[j + 1] - resist[j]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3;
    d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
    a[j] = resist[j];
  }
}

// IA generated for best fit 10 points to the curve
float interpolateTemperature(float R) 
{
  for (int i = 0; i < N - 1; i++) {
    float R0 = resist[i];
    float R1 = resist[i + 1];
    if ((R <= R0 && R >= R1) || (R >= R0 && R <= R1)) {
      float T0 = temps[i];
      float T = T0;
      for (int iter = 0; iter < 5; iter++) {
        float x = T - T0;
        float f = a[i] + b[i]*x + c[i]*x*x + d[i]*x*x*x - R;
        float df = b[i] + 2*c[i]*x + 3*d[i]*x*x;
        if (abs(df) < 1e-6) break;
        T -= f / df;
      }
      return T;
    }
  }
  return NAN;
}

// Use the ADC to measure voltage through the thermistor, from there get the resistance
// and use the spline to interpolate temperature
void readThermistorTemperature(SensorData &data)
{
  // data.temp = 22;
  // data.tempf = 22.5f;
 
  uint16_t adc0 = 0;
  float v = 1.0;
  float R = 1.0;
  float T = 0.0;

#ifdef ADC_POWERED_BY_PIN
  Adafruit_ADS1115 adc;
  Adafruit_ADS1115 &myAdc = adc; // Referece the local object
#else
  Adafruit_ADS1115 &myAdc = ads; // Referece the global object
#endif

   // Let's see if with this we save battery power
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  myAdc.begin();
  myAdc.setGain(GAIN_SIXTEEN); //+/- 0.256V  1 bit = 0.0078125mV 

  pinMode(THERMISTOR_POWER_PIN, OUTPUT);
  digitalWrite(THERMISTOR_POWER_PIN, HIGH);
  delay(100); 

  delay(THERMISTOR_CURRENT_TIME);
  adc0 = myAdc.readADC_Differential_0_1();
  
  pinMode(THERMISTOR_POWER_PIN, INPUT);
  digitalWrite(THERMISTOR_POWER_PIN, LOW);

#ifdef ADC_POWERED_BY_PIN  
  myAdc.conversionComplete();

  delay(500);
  digitalWrite(5, LOW);
  pinMode(5, INPUT);
#endif

  v = float(adc0) * multiplier; 
  R = (v / 1000) / CURRENT_A;
  T =interpolateTemperature( R );

  if(T==NAN)
    T = -3;

  data.temp = int(T);

  float Tf = (T - (int)T);
  data.tempf =  (int)(100*Tf);

#ifdef SHOW_SERIAL
  Serial.print("AIN0: "); 
  Serial.print(adc0);
  Serial.print(" V="); 
  Serial.print(v , 5);
  Serial.print(" R="); 
  Serial.print(R);
  Serial.print(" Tf="); 
  Serial.print(data.tempf);
  Serial.print(" T="); 
  Serial.println(T , 2);
#endif 
}

void setup() 
{
  delay(2000); // in case of continous crashes,try not to deplete battery  
  pinMode(RF_TX_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("sensor_huerto_sleep_thermistor"); 

  computeSplineCoefficients();

  Serial.println("Coefficients ok");

  // Config ADS1115 
 #ifndef ADC_POWERED_BY_PIN
 // OJO
    pinMode(5, OUTPUT);    // DEBE ESTAR SI NO HAY ADC ALIMENTADO POR PIN 5
    digitalWrite(5, HIGH);
  
    delay(100);
    ads.setGain(GAIN_SIXTEEN);    //+/- 0.256V  1 bit = 0.0078125mV 
    ads.begin();

    Serial.println("ADS1115 ok");
#endif
 // pinMode(8, OUTPUT);
 // digitalWrite(8, HIGH);

  delay(50);
  setup_wdt();
  delay(50);
  
  awakes = MAX_AWAKES;
  maxAwakes = MAX_AWAKES; 
  reset = 0; 
  powerState = POWER_STATE_HIGH ;
  powerSaveLevel = 0;
  bReboot = false;

  Serial.println("Initializing RF_TX"); 
  
  //Unused 
  //pinMode(LED_STATE, OUTPUT);

  mySwitch.enableTransmit(RF_TX_PIN);
  mySwitch.setRepeatTransmit(20);
     

#ifndef NO_SENSOR    
  Serial.println("BME280 initializing sensor board");
   
  boardOk = setupBoard(&bmeBoard,true,0x76);
  if(!boardOk)
  {
    Serial.println("Error initializing sensor board");
  }

  Serial.println("normal mode, 2x oversampling for all, filter off,");
  Serial.println("1000ms standby period");
    
  Serial.println();
#endif
  //pinMode(LED_STATE, INPUT);

  setup_wdt();

  delay(100);
}

void loop() 
{
  unsigned long voltage = 0;

  SensorData dat1 ={0,0,0};
  SensorData dat2 ={0,0,0};
   
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
    delay(50);

    return;
  }
    
  awakes = 0;
    
  //delay(5000);
 #ifdef SHOW_SERIAL
  Serial.print("Sensor Board: ");
 #endif   

  pinMode(BME_CS_BOARD, OUTPUT);    
  digitalWrite(BME_CS_BOARD, LOW);

  delay(delayTime); 

#ifndef NO_SENSOR
  boardOk = setupBoard(&bmeBoard,true,0x76);
  if(boardOk)
  {
    bmeBoard.takeForcedMeasurement();
    dat1 = getValues(bmeBoard); //  Fill the values
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

  bmeBoard.takeForcedMeasurement();
  delay(delayTime);  

  digitalWrite(BME_CS_BOARD, HIGH);
  pinMode(BME_CS_BOARD, INPUT);

  delay(delayTime);
  readThermistorTemperature(dat2);
  delay(delayTime); 
  
  //unsigned long data = set_message(0xA,dat1.temp,dat2.temp,dat1.humidity,dat2.humidity);
  unsigned long data = set_message3(0xC,dat1.temp,dat2.temp,dat1.humidity,dat2.tempf);
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


  unsigned long message = set_message2(0xD,avgVoltage,dat1.pressure,level);

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
    
  // Send RF data
    
  mySwitch.send(message, SIZE_BITS);
    
    //digitalWrite(11, LOW);
    delay(delayTime); 
    
    //pinMode(BME_SCK, INPUT);
    //pinMode(BME_CS_BOARD, INPUT);
   

    /* Re-enter sleep mode. */
    enterSleep();
}

// Get BME280 sensor values
SensorData getValues(Adafruit_BME280 &bme) 
{
  SensorData dat{0,0,0};

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
