/*
This server connects to one of the WiFis stored in an array and receives RF data from two sensorss.
It reads the RF packet identifiers to know how to read the messages, anf extract its data. Then, it
forwards the data to ThingerIO using a registered user.

Also, it has attached a cable comming from a hall effect flowmeter that produces an interrupt to measure 
water flow. (The formula is LiterPerMinute=number_pulses/6.6). The water pipe goes to a couple of irrigation
programmers that are manually programmed independently of all this code.  
With this code not only measures water flow, also controls how many minutes water is flowing, and if a limit 
is reached,it commands a solenoid valve to close water flow. This allows to detect an undesired condition 
in the water pipe, broken or stuck programmer. So, we have not to worry about an excessive water comsuption

To control the solenoid valve, a DC dual H-Bridge motor controller driver board base on MX1616 controller is
used. To use it, a couple of GPIO pins are needed to open or close the valve.
*/

//#define _DEBUG_
 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <RCSwitch.h>
#include <ThingerESP8266.h>

#include <functional>

RCSwitch mySwitch;

String gMyIP = "";

// I have two "servers", which is the max number of free devices in ThingerIO
#define SERVER_HOME
//#define SERVER_ORCHARD

// Blue onboard LED pin and RF reception interrupt pin
const uint8_t RF433_RX_MARK_PIN = 2;
const uint8_t RF433_RX_RECEIVE_PIN = 14;

// Red LED indicator and flowmeter interrupt pin
const uint8_t FLOWMETER_MARK_PIN = 13;
const uint8_t FLOWMETER_PIN = 12;

// Solenoid valve pins
const uint8_t VALVE_OPEN_PIN = 5;
const uint8_t VALVE_CLOSE_PIN = 4;

int32_t rfOldValue = 0;

const uint32_t SERIAL_SPEED     = 115200; ///< Set the baud rate for Serial I/O

// Test
// Create AsyncWebServer object on port 80
//AsyncWebServer server(80); 

/// List of wifi SSID's from which we check for an update
struct Wifi
{ 
  const char *ssid;
  const char *pwd;  
  const char *url;
};

struct Wifi wifis[3] =
{
  { "WIFI_!","XXXXXXXXX","http://192.168.1.45:7020/file.bin"},
  { "WIFI_2", "XXXXXXXXXX","http://192.168.1.45:7020/file.bin"},
  { "WIFI_3","123456789","http://192.168.43.107:7020/file.bin"}
};

// Tokens for ThingerIO obtained from online registration process
#ifdef SERVER_HOME
ThingerESP8266 thing("Boli", "DeviceIdCasa", "XYZ_Token");
#define HEADER_PKG_1 0xC
#define HEADER_PKG_2 0xD
#elif
ThingerESP8266 thing("Boli", "DeviceIdHuerto", "XYZ_Token");
#define HEADER_PKG_1 0xA
#define HEADER_PKG_2 0xB
#endif

bool connectWifi(struct Wifi wifi, int retries)
{
  int tries = retries;
  bool led = true;
  unsigned long last = 0;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifi.ssid);

  WiFi.begin(wifi.ssid, wifi.pwd);
  thing.add_wifi(wifi.ssid, wifi.pwd);

  // I have a red LED outside the box, to know we are in process of pairing to Wifi
  pinMode(FLOWMETER_MARK_PIN, OUTPUT);
  digitalWrite(FLOWMETER_MARK_PIN, led ? HIGH:LOW);

  while ( tries>=0 && (WiFi.status() != WL_CONNECTED) ) 
  { 
    //The delay of 500ms here makes led not able to flash.Stays always on ¿?
    // So we loop 450+50ms
    if(millis() - last > 450)
    {
      Serial.println(tries);

      tries--;
  
      if(WiFi.status() == WL_CONNECTED)
      {
        return true;
      }
      else
        thing.add_wifi(wifi.ssid, wifi.pwd);
       
      led = !led;
             
      digitalWrite(FLOWMETER_MARK_PIN, led ? HIGH:LOW);
      last = millis();      
    }
    delay(50); // Some delay is needed so Wifi has time to act
  }

  return (WiFi.status() == WL_CONNECTED);
}

// Try to connect to some configured wifi
void tryConnect()
{
  bool bConnected = false;
  int k = 0;
  
  do
  {
    for(int i=0; i < sizeof(wifis) ; i++)
    {       
      if(connectWifi(wifis[i],30))
      {
        Serial.println("Connected!!!");
        bConnected = true;
        IPAddress ip = WiFi.localIP();
        gMyIP = ip.toString(); 
        break;
      }
    }
    k++;
  }
  while(!bConnected ); 
}

unsigned long t = millis();
unsigned long tm = t; 


// TODO Group this variables instead of using globals
int tempSonda = -1;
int tempCable = -1;
int tempCasa = -1;
float tempPrecise = -1;
int humSonda = -1;
int humCable = -1;
int humCasa = -1;
int presSonda = -1;
int presCable = -1;
int presCasa = -1;
int voltage = -1;
int powerState = -1;
int lastRf = -1;
int lastRf2 = -1;
float flowValue = -1;

int tempSondaLast = -1;
int tempCableLast = -1;
int tempCasaLast = -1;
int humSondaLast = -1;
int humCableLast = -1;
int humCasaLast = -1;
int presSondaLast = -1;
int presCableLast = -1;
int presCasaLast = -1;
int voltageLast = -1;
int powerStateLast = -1;
int lastRfLast = -1;
int lastRf2Last = -1;

uint32_t lastRfValue = 0;
uint32_t lastRf2Value = 0;
unsigned long lastReception = 0;
unsigned long lastReception2 = 0;

// Configuration variables for flowmeter
const int Qmax = 1;      // Liters per minute max to consider a leak
const int Tmax = 20;      // Minutes max with open valve
const int Tcheck = 120;   // Minutes to wait ext check. 
const int Tverif = 6;    // Seconds to have open valve to verify leaks

// State variables to control irrigation state
enum State {
    WAITING_FOR_IRRIGATION, // Pipe closed.
    IRRIGATION_ACTIVE,      // Water flowing in the pipe when automatic water irrigation timer opens
    PIPE_FAILURE,           // Pipe has been providing water more that Tmax minutes
    VERIFYING_LEAK          // Checking: after detecting one failure, we open from some time to time to check if now we can open valve again
} current_state;

unsigned long irrigation_start_time = 0;
unsigned long pipe_failure_time = 0;
unsigned long verification_start_time = 0;
unsigned long flow_show_time = 0;
unsigned long valve_check = 0;

bool mustOpen = false;
bool mustClose = false;

#define FLOWRATE_SHOW 20  // Secons to show flow interval
#define NUM_SAMPLES 10   // Number of samples for averaging
#define VALVE_PULSE_DURATION 35 // Milliseconds to keep pulse high
#define VALVE_CHECK 12 * 60 // Each VALVE_CHECK minutes close and open to avoid valve sticked 

// Use interrupt to measure flow
volatile unsigned int pulseCount = 0;  // Pulse counter in ISR
void ICACHE_RAM_ATTR staticPulseCount() 
{
         pulseCount++;  // Just count pulses in ISR
}

// This class controls flow state machine
// Measure flow, and if too much time passed,close the solenoid valve
//
class FlowMeter {
  private:
    unsigned long previousTime = 0;
    const unsigned long measurementInterval = 1000;  // Measurement every second
    float accumulated = 0; // Liters since boot
    float lastFlowRate = 0;

    // Variables for average and standard deviation
    float flowSamples[NUM_SAMPLES] = {0};  // Circular buffer for last N measurements
    int sampleIndex = 0;
    bool bufferFilled = false;

    float flowAverage = 0;
    float flowStdDev = 0;

  public:
    FlowMeter() {
        pinMode(FLOWMETER_PIN, INPUT);
    }

    void updateFlowRate() {
        unsigned long currentTime = millis();
        if (currentTime - previousTime >= measurementInterval) {
            previousTime = currentTime;
            float frequency = pulseCount;
            lastFlowRate = frequency / 6.6;  // Convert to liters per minute
            pulseCount = 0;

            // Store the new sample
            flowSamples[sampleIndex] = lastFlowRate;
            sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

            if (sampleIndex == 0) {
                bufferFilled = true;  // Once we wrap around, we have enough samples
            }

            // Liters per second, times the second
            accumulated += (lastFlowRate / 60)* (measurementInterval / 1000);

            // Compute average and standard deviation
            computeStatistics();
        }
    }

    void computeStatistics() {
        float sum = 0, sumSq = 0;
        int count = bufferFilled ? NUM_SAMPLES : sampleIndex;

        for (int i = 0; i < count; i++) {
            sum += flowSamples[i];
            sumSq += flowSamples[i] * flowSamples[i];
        }

        flowAverage = sum / count;
        flowStdDev = sqrt((sumSq / count) - (flowAverage * flowAverage));
    }

    float getFlowRate() const {
        return lastFlowRate;
    }

    float getFlowAverage() const {
        return flowAverage;
    }

    float getFlowStdDev() const {
        return flowStdDev;
    }

    unsigned long getAccumulated()
    {
      return (accumulated * 10) / 10;
    }
};

// Utility methods to control solenoid valve using GPIOs
void openValve() {
    digitalWrite(VALVE_CLOSE_PIN, LOW);

    digitalWrite(VALVE_OPEN_PIN, HIGH);
    delay(VALVE_PULSE_DURATION);
    digitalWrite(VALVE_OPEN_PIN, LOW);
    
    Serial.println("Valve opened.");
}

void closeValve() 
{
    digitalWrite(VALVE_OPEN_PIN, LOW);

    digitalWrite(VALVE_CLOSE_PIN, HIGH);
    delay(VALVE_PULSE_DURATION);
    digitalWrite(VALVE_CLOSE_PIN, LOW);

    Serial.println("Valve closed");
}

void controlValve(bool in)
{
  if(in)
    openValve();
  else
    closeValve();  
}

void setupIrrigation() 
{
    attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), staticPulseCount, RISING);
   
    pinMode(VALVE_OPEN_PIN, OUTPUT);
    pinMode(VALVE_CLOSE_PIN, OUTPUT);
    
    // Test to hear the opening three times
    closeValve();
    delay(1000);
    openValve();  
    closeValve();
    delay(1000);
    openValve();
    closeValve();
    delay(1000);
    openValve();  // The valve starts open    
    
    current_state = WAITING_FOR_IRRIGATION;
}

// Static instance initialization;
static FlowMeter instance;

// Control irrigation function
void loopIrrigation() 
{
    instance.updateFlowRate();
    float flowRate = instance.getFlowRate();

    flowValue = flowRate;
    //Serial.print("Flow rate (L/min): ");
    //Serial.println(flowRate);
    unsigned long now = millis();

    switch (current_state) 
    {
        case WAITING_FOR_IRRIGATION:
            if (flowRate > 0) 
            {  // Detects the start of irrigation
                irrigation_start_time = now;
                current_state = IRRIGATION_ACTIVE;
            }

            break;

        case IRRIGATION_ACTIVE:
            if (flowRate == 0) 
            {
                current_state = WAITING_FOR_IRRIGATION;  // Irrigation has ended
            } 

            if (now - irrigation_start_time > Tmax * 60000) 
            {
                closeValve();
                pipe_failure_time = now;
                current_state = PIPE_FAILURE;
            }
            
            break;

        case PIPE_FAILURE:
            Serial.println("Leak detected. Valve closed.");
            
            if (now - pipe_failure_time > Tcheck * 60000) 
            {
                openValve();
                verification_start_time = now;
                current_state = VERIFYING_LEAK;         
            }
            
            break;

        case VERIFYING_LEAK:
            if (now - verification_start_time > Tverif * 1000) 
            {
                if (flowRate > Qmax) 
                {
                    closeValve();
                    pipe_failure_time = now;  // Reset leak detection time
                    current_state = PIPE_FAILURE;
                } 
                else 
                {
                    current_state = WAITING_FOR_IRRIGATION;                   
                }
            }      

            break;
    }

    if(now - flow_show_time > FLOWRATE_SHOW * 1000)
    {
      flow_show_time = now;

      Serial.print("ActualFlow=");
      Serial.print(instance.getFlowRate());
      Serial.print("L/min AverageFlow=");
      Serial.print(instance.getFlowAverage());
      Serial.print("L/min (");
      Serial.print(instance.getFlowStdDev());
      Serial.println(")");
    }

    if(mustOpen)
    {
      mustClose = false;

      closeValve();  
      openValve();
            
      mustOpen = false;
    }

    if(mustClose)
    {
      mustOpen = false;
      
      openValve();  
      closeValve();

      mustClose = false;
    }

    if(now - valve_check > VALVE_CHECK * 60000)
    {
      valve_check = now;
      
      closeValve();
      delay(500);
      openValve();      
    }
}

int getIrrigationState()
{
  int st = 5;

  switch(current_state)
  {
    case WAITING_FOR_IRRIGATION: st = 0; break;
    case IRRIGATION_ACTIVE: st = 1; break;
    case PIPE_FAILURE: st = 2;break;
    case VERIFYING_LEAK: st = 3; break;
    default: st = 5; break;
  }

  return st;
}

String getState()
{
  String st = "";
  float timeleft = 0;
  unsigned long x = 0;

  unsigned long now = millis();
  
  const int Qmax = 1;      // Liters per minute max to consider a leak
  const int Tmax = 20;      // Minutes max with open valve
  const int Tcheck = 120;   // Minutes to wait ext check. 
  const int Tverif = 6;    // Seconds to have open valve to verify leaks
  
  String data = String("\nTnmax=") + String(Tmax) + String(" minutes  Tcheck=") + String(Tcheck) + String(" minutes  Tverif=") + String(Tverif); 
  switch(current_state)
  {
    case WAITING_FOR_IRRIGATION: 
      st = "WAITING_FOR_IRRIGATION";     
      break;
    case IRRIGATION_ACTIVE: 
      st = "IRRIGATION_ACTIVE";
      break;
    case PIPE_FAILURE: 
      st = "PIPE_FAILURE. Time left for VERIFYING_LEAK=";break;
      timeleft = (Tcheck * 60000 - (now - pipe_failure_time))/60;

      x = (timeleft*10)/10;
      st = String(st) + String(x);
    case VERIFYING_LEAK: 
      st = "VERIFYING_LEAK";
      break;
    default: st = "UNKONW"; 
  }

  st = String(st) + String("\nFlow=") + String(flowValue) + String(" L/min\nAverage=") + String(instance.getFlowAverage()) + String(" L/min\nAccumulated=") + String(instance.getAccumulated()) + String(" L\n"); 
  st += data;

  return st;
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
    //uint32_t result = ((uint32_t)(header & 0x03)) << 30;

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

    // First, get header and return it to know how decoding must proceed
    //uint8_t header = (packed >> 30) & 0x03;
    uint8_t header = (packed >> 28) & 0x0F;

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

void setup()
{
  Serial.begin(SERIAL_SPEED);
  #ifdef  __AVR_ATmega32U4__  // If this is a 32U4 processor, then wait 3 seconds for the interface to initialize
    delay(3000);
  #endif
  
  mySwitch.enableReceive(RF433_RX_RECEIVE_PIN);  // Receiver input on interrupt 0 (D2)
  pinMode(RF433_RX_MARK_PIN, OUTPUT); // Light a LED to mark data received 
  digitalWrite (RF433_RX_MARK_PIN, HIGH); 
 
  Serial.println(F("RF433 initialized\n"));

  setupIrrigation();

  tryConnect();
    
  // Print ESP8266 Local IP Address
  Serial.println(WiFi.localIP());

  //thing["valve"] << controlValve();

  thing["temp"] >> [](pson& out)
  {
    Serial.print("Sending T=");
    Serial.print(tempSonda);
    Serial.print(" Tc=");
    Serial.print(tempCable);
    Serial.print(" Th=");
    Serial.print(tempCasa);
    Serial.print(" Tf=");
    Serial.print(tempPrecise, 2);
    
    Serial.print(" Hs=");
    Serial.print(humSonda);
    Serial.print(" Hc=");
    Serial.print(humCable);
    Serial.print(" Hh=");
    Serial.print(humCasa);
    
    Serial.print(" Ps=");
    Serial.print(presSonda);
    Serial.print(" Pc=");
    Serial.print(presCable);
    Serial.print(" Ph=");
    Serial.print(presCasa);

    Serial.print('\n');
    
    if(tempPrecise != -1)
    {
        out["Tp"] = tempPrecise;
    }
    else
      out["Tp"] = 0;

    if(tempSonda != -1)
    {
      out["T"] = tempSonda;
    }
    else
     out["T"] = 0;

    if(tempCable != -1)
    {
      out["Tc"] = tempCable;
    }
    else
     out["Tc"] = 0;

    if(tempCasa != -1)
    {
      out["Th"] = tempCasa;
    }
    else
     out["Th"] = 0;

    if(humSonda != -1)
    {
      out["Hs"] = humSonda;
    }
    else
     out["Hs"] = 0;

    if(humCable != -1)
    {
      out["Hc"] = humCable; 
    }
    else
     out["Hc"] = 0;

    if(humCasa != -1)
    {
      out["Hh"] = humCasa;
    }
    else
     out["Hh"] = 0;

    if(presSonda != -1)
    {
      out["Ps"] = presSonda;
    }
    else
     out["Ps"] = 0;
      
    if(presCable != -1)
    {
      out["Pc"] = presCable;
    }
    else
     out["Pc"] = 0;

    if(presCasa != -1)
    {
      out["Ph"] = presCasa;
    }
    else
     out["Ph"] = 0;

    if(voltage != -1)
    {
      out["Vb"] = voltage;
    }
    else
     out["Vb"] = 0;
     
     out["S"] = powerState;
     out["L"] = lastRf;
     out["L2"] = lastRf2;
     out["A"] = gMyIP;
     out["Fi"] = instance.getFlowRate(); //Instantaneous flow rate
     out["Fa"] = instance.getAccumulated(); 
     out["Sr"] = getIrrigationState();
  };
  
  lastRf = 0;
  lastRf2 = 0;
  lastReception = millis();
  lastReception2 = lastReception;
} 

void show(const char*id,float value)
{
  Serial.print(id);
  Serial.print(":");
  Serial.print((float)value,1);
  Serial.print('\n');
}

void flashLed()
{
  // Onboard led 4
    digitalWrite (RF433_RX_MARK_PIN, LOW); // Making LED High.
    delay(50);              // Some Delay
    digitalWrite (RF433_RX_MARK_PIN, HIGH);  // Making LED LOW.
}

void loop()
{
  uint32_t rfValue = 0;
  
  unsigned long now = millis();

  thing.handle();

  //digitalWrite (RF433_RX_MARK_PIN, HIGH); 
   
  if (mySwitch.available())
  {
    rfValue = mySwitch.getReceivedValue() ;
    mySwitch.resetAvailable();
    
    // Avoid too much repetitions
    //if(rfValue == lastRfValue)
    //{
    //  return;
    //}
    
    Serial.print("Received ");
    Serial.print(rfValue);
 
    ////////////////////////////////////////////////////////////////
    /// First message: temp Thremistor,temp BME280, power state
    ////////////////////////////////////////////////////////////////
     
    Field tempF(14, true, Float);     // −20.00 to +50.00 with 2 decimal
    Field tempSensorF(7, true, Integer);  // -64 to 64
    Field stateF(2, false, Integer);     // No sign,0 to 4. 2 bit

    Field fields1[3] = { tempF, tempSensorF, stateF };

    //MessagePacker::inspectMessage(rfValue, fields1, 3);
  
    Value values1[4];
    uint8_t header = MessagePacker::unpack(rfValue, fields1, values1, 3);

    Serial.print(" Header = ");
    Serial.println(header);

    if(header == 0x0F) //0x02)
    {
      MessagePacker::inspectMessage(rfValue, fields1, 3);

      tempPrecise = values1[0].floatValue;
      tempSonda = values1[1].intValue;
      powerState = values1[2].intValue;

      tempCable = MessagePacker::roundFloat(tempPrecise);
      
      lastReception = now;

      flashLed();

      Serial.print("Tprecise="); 
      Serial.print(tempPrecise,2); 
      Serial.print(" T="); 
      Serial.print(tempSonda); 
      Serial.print(" S="); 
      Serial.println(powerState);
    }
    else if(header == 0x0E) //0x03)
    {       
      /////////////////////////////////////////////////////////////
      // Second message Batery voltage, pressure, humidity
      /////////////////////////////////////////////////////////////

      Field voltageF(13, false, Integer);  // No sign,0 to 8192mV. Plenty of bits.
      Field presF(8, true, Integer);    // −128 to +128
      Field humF(7, false, Integer);    // 0 to 100

      Field fields2[3] = { voltageF, presF, humF};
      
      // unpack
      Value values2[4];
      uint8_t header = MessagePacker::unpack(rfValue, fields2, values2, 3);

      voltage = values2[0].intValue;
      presSonda = 1000 + values2[1].intValue;
      humCable = values2[2].intValue;

      lastReception = now;

      flashLed();

      Serial.print(" V=");
      Serial.print(voltage);
      Serial.print(" Pres=");
      Serial.print(presSonda);
      Serial.print(" Hum=");
      Serial.println(humCable);
    }
    else
    {
      // Nothing interesting for us received.
      lastRf = (now - lastReception)/1000;
      lastRf2 = (now - lastReception2)/1000; // From home
    
      return;
    }    

    /*
    else if(check_header(0x9,rfValue))
    {  
      lastRf2Value = rfValue;
      unsigned long first = get_val(rfValue,0,false);
      if(first != 127)
      {
        Serial.println("HEADER 0x9 noise ");
        //Not the message we are waiting
      }
      else
      {
        Serial.println("HEADER 0x9 ");

        humCasa = get_val(rfValue,1,false);
        tempCasa = get_val(rfValue,2,true);
        presCasa = 1000 + get_val(rfValue,3,true);
 
        if((tempCasaLast != -1) && abs(tempCasa - tempCasaLast)>5)
        {
          tempCasa = tempCasaLast;
          tempCasaLast = -1; 
        } 
        else
        {
          tempCasaLast = tempCasa;          
        }

        if((humCasaLast != -1) && abs(humCasa - humCasaLast)>10)
        {
          humCasa = humCasaLast;
          humCasaLast = -1; 
        } 
        else
        {
          humCasaLast = humCasa;
        }

        if(humCasa == 1 || humCasa == 2)
        {
          // Special values if there are problems
          humCasa = 0; 
        }

        Serial.print("humCasa: ");
        Serial.println(humCasa);

        Serial.print("tempCasa: ");
        Serial.println(tempCasa);
      
        Serial.print("presCasa: ");
        Serial.println(presCasa);

        //lastReception = now;
        lastReception2 = now;

        flashLed();
      }
    }
    else
    {
      // Nothing received.
      lastRf = (now - lastReception)/1000;
      lastRf2 = (now - lastReception2)/1000;
      
      return;
    }
    */
    Serial.println('\n');  

    // Onboard led 4
    digitalWrite (RF433_RX_MARK_PIN, LOW); // Making LED High.
    delay(50);              // Some Delay
    digitalWrite (RF433_RX_MARK_PIN, HIGH);  // Making LED LOW.
  }

  lastRf = (now - lastReception)/1000;
  if( lastRf > 35*60) //35 min with no RF contact
  {
    tempSonda = -1;
    tempCable = -1;
    humSonda = -1;
    humCable = -1;
    presSonda = -1;
    presCable = -1;
    voltage = -1; 
  }
  
  lastRf2 = (now - lastReception2)/1000; 
  if( lastRf2 > 35*60) //35 min 
  {
    tempCasa = -1;
    humCasa = -1;
    presCasa = -1;
  }

  delay(150);  
  
  loopIrrigation();

  if((WiFi.status() != WL_CONNECTED))
  {
    Serial.print("REBOOT !!");
    ESP.restart();
    return;
  }

     

} // of method loop()

