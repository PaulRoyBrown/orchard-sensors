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
//#define ADC_POWERED_BY_PIN // A try to see if I can reduce power comsuption by switching ADC with a GPIO

/////////////////////////////
// RCSwitch to transmit data
/////////////////////////////

#define RF_POWER_PIN 9
#define RF_TX_PIN 4
#define MSG_SIZE_BITS 32  // 32bit each message to use uint32_t

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
#define THERMISTOR_CURRENT_UA   103.02 //30.89 // Constant current µA. Important to be well measured!!
#define THERMISTOR_CURRENT_TIME 100

// Calibrated table of temp/resistance thermistor curve
// Values between -5 and 45 temperature degrees
const int N = 11;
const float temps[N] = {-5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45};
const float resist[N] = {12300, 9423, 7282, 5672, 4450, 3515, 2796, 2238, 1802, 1459, 1188};


// We measure the thermistor voltage with this ADC chip
Adafruit_ADS1115 ads; //Global object

bool bData = false; // Changed to true in ALERT interrupt to say there is data available 

const float CURRENT_A = THERMISTOR_CURRENT_UA / 1e6; // Convert to amps

// The max generated voltage drop in thermistor depends on the current choosen.
 //For 100uA max voltage could be according to resist[] array 12300 * 100uA =  1.25V 
// so according to Table 7-1. Full-Scale Range and Corresponding LSB Size in 
// idatasheet we must choose max range FSR=2048 => multiplier = 0.0625F 
// For 30uA generated with LM334 we are ok in first step. (0.0078125F)

// Scale factor ADC
float multiplier =  0.0625F ; //0.0078125F; 

// GAIN must be according to the FSR. Taken from the header Adafruit_ADS1X15.h
adsGain_t THERMISTOR_ADC_GAIN = GAIN_TWO; //GAIN_SIXTEEN

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

uint8_t sleepDuration = 2;
volatile uint8_t awakes = MAX_AWAKES; // 8, so first time in loop,we send information to make checks easier (do not wait 60 secs) after power on
volatile uint8_t maxAwakes = MAX_AWAKES; // This is increased if low voltage to save batery
volatile uint8_t reset = 0; 

// Battery limits to start saving power
/*
#define VOLTAGE_HIGH    4099
#define VOLTAGE_MEDIUM  4061//4050//4010
#define VOLTAGE_LOW     4050 //4000//3959
#define VOLTAGE_ULTRALOW  3995 // 3770//3950
*/

#define VOLTAGE_HIGH    4540
#define VOLTAGE_MEDIUM  4520//4050//4010
#define VOLTAGE_LOW     4500 //4000//3959
#define VOLTAGE_ULTRALOW  4 // 3770//3950

// States that match above battery limits
#define POWER_STATE_HIGH    1  // 1*8*MAX_AWAKES sec -> aprox 8*8 = 1 min
#define POWER_STATE_MEDIUM  3  // 3*8*MAX_AWAKES sec -> aprox 3*8*8 = 3 min aprox
#define POWER_STATE_LOW     10 // 10*8*MAX_AWAKES sec -> aprox 10*8*8 = 10 min aprox
#define POWER_STATE_ULTRALOW   30 // 30*8*MAX_AWAKES sec -> aprox 30*8*8 = 30 min aprox

uint8_t powerState = POWER_STATE_HIGH ; // ULTRALOW,LOW,MEDIUM,HIGH Actual power state
uint8_t powerSaveLevel = 0;
bool bReboot = false;

// Interrupt coming from ALERT ADS1115 pin
void adcISR()
{
  bData = true;
  
  //Serial.println("INTR");
}

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
#define MAX_AVG_SAMPLES 5
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
  if(sleepDuration == 2)
  {
    WDTCSR = (1 << WDIE) | (1 << WDP0) | (1 << WDP1) | (1 << WDP2);
  }
  else
  {
     // Configure: 8 sec + interrupt, not reset
    WDTCSR = (1 << WDIE) | 1<<WDP0 | 1<<WDP3; 
  }

  //MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  //MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above

  sei();           // Reset interrupts handling
}

/**
 Discriminator for the C type of data a value stores 
*/
enum ValueType {
  Integer,
  Float
};

/**
Descriptor of a given Field that says how many bits a Value has
*/
struct Field {
  uint8_t bits;
  bool isSigned;
  ValueType type;

  Field(uint8_t b, bool s, ValueType t = Integer)
    : bits(b), isSigned(s), type(t) {}
};

/** 
  A Value to be stored and transmitted/received by RF. 
  - A Value can be stored as a int or a float, and type says which it is
  - Field stores the bits ocuppied by this Value (0 to 32bit)
   TODO(Paul) use a union for this, to optimize size  
*/
struct Value {
  Field field;
  ValueType type;
  int32_t intValue;
  float floatValue;

  Value() : field(0, false, Integer), type(Integer), intValue(0), floatValue(0.0f) {}

  Value(int32_t v, Field f) : field(f), type(Integer), intValue(v), floatValue(0.0f) {}
  Value(float v, Field f) : field(f), type(Float), intValue(0), floatValue(v) {}
};

/**
  A class to pack and unpack an array of Values, each one described by its descriptor
  Floats are sent as integers and changed to float in receiving side by dividing by proper factor.(now 100)
  
  NOTE:This class is exactly the same, by copy&paste, in the RF sending and reception sides.

  TODO(Paul): Make the number of decimals to be transmitted customizable.Now, restricted to two decimals.
              Also the number of bits used for header (now 2)  
*/
class MessagePacker {
public:
  static int32_t roundFloat(float f) {
    return (f >= 0) ? (int32_t)(f + 0.5f) : (int32_t)(f - 0.5f);
  }

  static void validateRuntime(const Value& v) {
    int32_t raw = (v.type == Float)
      ? roundFloat(v.floatValue * 100.0f)
      : v.intValue;

    int32_t max = v.field.isSigned ? (1 << (v.field.bits - 1)) - 1 : (1 << v.field.bits) - 1;
    int32_t min = v.field.isSigned ? -(1 << (v.field.bits - 1)) : 0;
    if (raw < min || raw > max) 
    {    
      Serial.print("⚠️ Value out of range: "); // Always print it
      Serial.println(raw);      
    }
  }

  static uint32_t encode(const Value& v) 
  {
    validateRuntime(v);

    int32_t raw = (v.type == Float)
      ? roundFloat(v.floatValue * 100.0f)
      : v.intValue;

    // Generate a mask with as many '1' as number of bits in each field (-1) because bit shihts start with zero 
    uint32_t mask = (1UL << v.field.bits) - 1;

    if (v.field.isSigned) 
    {
      // Create mask with 1 in sign position
      uint32_t signBit = (raw < 0) ? (1UL << (v.field.bits - 1)) : 0; 
      // example for 5bit field: Generate 1UL << 5 = 0b00100000. Then, 0b00100000 - 1 = 0b00011111 is the value without sign 
      uint32_t magnitude = (uint32_t)(abs(raw)) & ((1UL << (v.field.bits - 1)) - 1); 
      return signBit | magnitude;
    } 
    else 
    {
      return (uint32_t)(raw) & mask; //Get only the bits in the field for this value
    }
  }

  static int32_t decode(uint32_t packed, const Field& f) 
  {
    uint32_t mask = (1UL << f.bits) - 1; // Mask to isolate value magnitude
    packed &= mask;

    if (f.isSigned) 
    {
      // Generate mask to have a '1' exactly at sign bit
      uint32_t signBit = 1UL << (f.bits - 1); 
      bool isNegative = packed & signBit;
      
      // Sign bit minus 1 set '1s' in all bits before sign bit, which is what we need to get the value magnitude 
      int32_t magnitude = packed & (signBit - 1);
      return isNegative ? -magnitude : magnitude; // Notice we return a signed type to include sigh
    } 
    else 
    {
      return (int32_t)(packed);
    }
  }

  static float decodeFloat(int32_t raw) 
  {
    return ((float)raw) / 100.0f;
  }

  static uint32_t pack(uint8_t header, Value values[], uint8_t count) 
  {
    uint8_t totalBits = 0;
    for (uint8_t i = 0; i < count; ++i) 
    {
      totalBits += values[i].field.bits;
    }

    if (totalBits > 28) //30) // 28 for 4bit header 
    {
      Serial.print("Error: total bits = "); // Always print it
      Serial.println(totalBits);

      return 0U;
    }

    // Move header bits at the beginning of the 32bit value
    //uint32_t result = ((uint32_t)(header & 0x03)) << 30; // For 2bit header
    uint32_t result = ((uint32_t)(header & 0x0F)) << 28; //For 4bit header

    uint8_t offset = 0;

  // For each field, encode and move its number of field. then move ahead its size
    for (uint8_t i = 0; i < count; ++i) 
    {
      result |= encode(values[i]) << offset;
      offset += values[i].field.bits;
    }

    return result;
  }

  static uint8_t unpack(uint32_t packed, Field fields[], Value values[], uint8_t count) 
  {
    uint8_t offset = 0;

    //uint8_t header = (packed >> 30) & 0x03;
    uint8_t header = (packed >> 28) & 0x0F;

    // First, get header and return it
    Serial.print("Header = ");
    Serial.println(header);

    // Now go for each bit field
    for (uint8_t i = 0; i < count; ++i) 
    {
      uint32_t mask = (1UL << fields[i].bits) - 1;
      uint32_t raw = (packed >> offset) & mask;
      int32_t decoded = decode(raw, fields[i]);

      if (fields[i].type == Float) 
      {
        values[i] = Value(decodeFloat(decoded), fields[i]);
      } 
      else 
      {
        values[i] = Value(decoded, fields[i]);
      }

      offset += fields[i].bits;
    }

    return header;
  }

  static void inspectMessage(uint32_t packed, Field fields[], uint8_t count) {  
    Value values[4];
    unpack(packed, fields, values, count);

    Serial.println("\n🔍 Message:");
    Serial.print("Header = ");

    //Serial.println((packed >> 30) & 0x03);
    Serial.println((packed >> 28) & 0x0F);

    for (uint8_t i = 0; i < count; ++i) {
      Serial.print("Campo ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(fields[i].type == Float ? "Float" : "Integer");
      Serial.print(", ");
      Serial.print(fields[i].isSigned ? "Signed" : "Unsigned");
      Serial.print(", ");
      Serial.print(fields[i].bits);
      Serial.print(" bits, Valor = ");
      if (fields[i].type == Float) {
        Serial.println(values[i].floatValue, 2);
      } else {
        Serial.println(values[i].intValue);
      }
    }   
  }
};

/*
head |voltage | press| state | reserved
|1011 |  13bit | 7bit | 2bit  | 000
pressure has sign because we have substracted 1000 and could be negative
*/

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
float readThermistorTemperature()
{
  // data.temp = 22;
  // data.tempf = 22.5f;
 
  uint16_t adc0 = 0;
  float v = 1.0;
  float R = 1.0;
  float T = 0.0;

#ifdef ADC_POWERED_BY_PIN
  Adafruit_ADS1115 adc;
  Adafruit_ADS1115 &myAdc = adc; // Reference the local object

  // Feed the ADC
  // Let's see if with this we save battery power
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  myAdc.begin();
  myAdc.setGain(GAIN_SIXTEEN); //+/- 0.256V  1 bit = 0.0078125mV 
#else
// ADC is feeded from setup so no need to do it here  
  Adafruit_ADS1115 &myAdc = ads; // Referece the global object

  digitalWrite(5, HIGH);
#endif


  pinMode(THERMISTOR_POWER_PIN, OUTPUT);
  digitalWrite(THERMISTOR_POWER_PIN, HIGH);
  delay(100); 

  delay(THERMISTOR_CURRENT_TIME);
  
  adc0 = myAdc.readADC_Differential_0_1();
/*
  myAdc.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false); //This jumps to 440uA

  unsigned long n = 0;
  while(!bData)
  {
    delay(200);
  #ifdef SHOW_SERIAL
    Serial.print("Retry n=");
    Serial.println(n);
    n++;
  #endif
  }

  bData = false;
  if(myAdc.conversionComplete())
  {
    // Read the conversion results
    adc0 =  myAdc.getLastConversionResults();
  }
  */
  digitalWrite(THERMISTOR_POWER_PIN, LOW);
  pinMode(THERMISTOR_POWER_PIN, INPUT);

#ifdef ADC_POWERED_BY_PIN  
  myAdc.conversionComplete();

  delay(500);
  digitalWrite(5, LOW);
  pinMode(5, INPUT);
#endif

  v = float(adc0) * multiplier; 
  R = (v / 1000) / CURRENT_A;
  T = interpolateTemperature( R );

  if(T==NAN)
    T = -3;

#ifdef SHOW_SERIAL
  Serial.print("AIN0: "); 
  Serial.print(adc0);
  Serial.print(" V="); 
  Serial.print(v , 5);
  Serial.print(" R="); 
  Serial.print(R);
  Serial.print(" T="); 
  Serial.println(T , 2);
#endif 

  return T;
}

void setup() 
{
  delay(2000); // in case of continous crashes,try not to deplete battery  
  pinMode(RF_TX_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println("sensor_huerto_sleep_thermistor"); 

  computeSplineCoefficients();
  attachInterrupt(digitalPinToInterrupt(3), adcISR, RISING);  
  
  Serial.println("Coefficients ok");

  // Config ADS1115 
 #ifndef ADC_POWERED_BY_PIN
    // Waych out!. This is due to my prototype board wiring. Should not be needed but it is for now!
    pinMode(5, OUTPUT);  
    digitalWrite(5, HIGH);
  
    delay(100);
    ads.setGain(THERMISTOR_ADC_GAIN);    //+/- 0.256V  1 bit = 0.0078125mV 
    ads.begin();

    Serial.println("ADS1115 ok");
#endif


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

  mySwitch.enableTransmit(RF_TX_PIN);
  mySwitch.setRepeatTransmit(10);
     

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

  setup_wdt();

  delay(100);
}

void loop() 
{
  unsigned long voltage = 0;

  SensorData dat1 ={0,0,0};
   
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
    
  delay(delayTime);
  
  /////////////////////////////////////////////////
  /// First message: temp,pressure,humidity 
  /////////////////////////////////////////////////
  
  float T = readThermistorTemperature();
  delay(delayTime); 

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

/*  
  // Send the temperature, pressure and humidity with first 0x02 message.
  // Definición de campos
  Field tempF(14, true, Float);     // With sign, −20.00 to +50.00 Temp with 2 decimal
  Field presF(8, true, Integer);    // With sign, −128 to +128 (1000 is substracted to pressure so we go from 872mb to 1128mb)
  Field humF(7, false, Integer);    // No sign, 0% to 100%

  Field fields1[3] = { tempF, presF, humF };

  // Pack into 32bit ulong
  Value temp((float)T, tempF);
  Value pres((int32_t)dat1.pressure, presF);
  Value hum((int32_t)dat1.humidity, humF);

  Value values1[3] = { temp, pres, hum };
*/
  Field tempF(14, true, Float);     // −20.00 to +50.00 with 2 decimal
  Field tempSensorF(7, true, Integer);     // With sign, -20 to +50 no decimals=> -64/+64 => 6bit + 1bit for sign
  Field stateF(2, false, Integer);     // No sign,0 to 4. 2 bit

  Value temp((float)T, tempF);
  Value tempSensor((int32_t)dat1.temp, tempSensorF);
  Value states((int32_t)level, stateF);

  Value values1[3] = { temp, tempSensor, states };

  // Pack it with msg id=3
  uint32_t data1 = MessagePacker::pack(0x0F, values1, 3);
  
  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, HIGH);
  mySwitch.send(data1, MSG_SIZE_BITS);
  

#ifdef SHOW_SERIAL 
  //show_bits_long(data);
  Serial.print("ValueSent: ");
  Serial.println(data1);
#endif

   /////////////////////////////////////////////////
  /// Second message ; Voltage,PowerState, temp
  /////////////////////////////////////////////////


  // Marker in graphs if we just come from a reboot
  if(bReboot)
  {
    level = 4;  // a momentary power state=4 should appear in graph just before reboot
    bReboot = false;    
  }

/*
  //unsigned long message = set_message2(0xD,avgVoltage,dat1.pressure,level);
  Field voltageF(13, false, Integer);  // No sign,0 to 8192mV. Plenty of bits.
  Field stateF(2, false, Integer);     // No sign,0 to 4. 2 bit
  Field tempSensorF(7, true, Integer);     // With sign, -20 to +50 no decimals=> -64/+64 => 6bit + 1bit for sign
 
  Field fields2[3] = { voltageF, stateF, tempSensorF };

  // Pack into 32bit ulong
  Value volts((int32_t)avgVoltage, voltageF);
  Value states((int32_t)level, stateF);
  Value tempSensor((int32_t)dat1.temp, tempSensorF);

  Value values2[3] = { volts, states, tempSensor };
*/
 
  Field voltageF(13, false, Integer);  // No sign,0 to 8192mV. Plenty of bits.
  Field presF(8, true, Integer);    // −128 to +128
  Field humF(7, false, Integer);    // 0 to 100

  Value volts((int32_t)avgVoltage, voltageF);
  Value pres((int32_t)dat1.pressure, presF);
  Value hum((int32_t)dat1.humidity, humF);
  
  Value values2[3] = { volts, pres, hum};
  
  // Pack it with id=4
  uint32_t data2 = MessagePacker::pack(0x0E, values2, 3);

  level = 0;
  
#ifdef SHOW_SERIAL   
  Serial.print("\nVcc is ");
  Serial.print(avgVoltage>0?avgVoltage:voltage);
  Serial.print(" mV state=");
  Serial.println(powerState);
  Serial.print("ValueSent:");
  Serial.print(data2);
  Serial.print("\n");
#endif
    
  // Send RF data message with PKT_SIZE_BITS size long
    
  mySwitch.send(data2, MSG_SIZE_BITS);
  
  digitalWrite(RF_POWER_PIN, LOW);

    //digitalWrite(11, LOW);
    delay(delayTime); 
    
    //pinMode(BME_SCK, INPUT);
    pinMode(BME_CS_BOARD, INPUT);
   
  digitalWrite(5, LOW); // With this, goes from 445uA to 280uA
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
  Serial.print("\t\t"); 
#endif

  dat.temp = (uint8_t)temp; //trunc(temp*100)/100;
  dat.humidity = (uint8_t)humidity;
  dat.pressure = (pressure - 1000);

  return dat;
#endif
}
