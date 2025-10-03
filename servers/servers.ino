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
#define _DEBUG_
 
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
//#include <ESPAsyncTCP.h>
//#include <ESPAsyncWebSrv.h>
#include <RCSwitch.h>
#include <ThingerESP8266.h>

#include <functional>

RCSwitch mySwitch;

String gMyIP = "";

// Registered user and device in ThingerIO (name,token)
ThingerESP8266 thing("Boli", "DeviceIdCasa", "ZZZZ");

// A different registered device for same user
//ThingerESP8266 thing("Boli", "DeviceIdHuerto", "ZZZZZ);

const uint8_t RF433_RX_MARK_PIN = 2;
const uint8_t RF433_RX_RECEIVE_PIN = 14;
const uint8_t FLOWMETER_MARK_PIN = 13;
const uint8_t FLOWMETER_PIN = 12;
const uint8_t VALVE_OPEN_PIN = 5;
const uint8_t VALVE_CLOSE_PIN = 4;

int32_t rfOldValue = 0;

const uint32_t SERIAL_SPEED     = 115200; ///< Set the baud rate for Serial I/O

int i = 0;
//uint8_t samples = 0;

const uint32_t MAXVALUES = 10;

float values[MAXVALUES] = {0}; 
float avg = 0;



/// List of wifi SSID's to which we connect, and from which we check for an update
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

bool connectWifi(struct Wifi wifi, int retries)
{
  int tries = retries;
  bool led = true;
  unsigned long last = 0;
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifi.ssid);

  WiFi.begin(wifi.ssid, wifi.pwd);
  thing.add_wifi(wifi.ssid, wifi.pwd);

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
    for(int i=0; i < 3 ; i++)
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

uint32_t samples[60];
uint32_t mtotal = 0;
uint32_t num_samples = 0;


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

// Just a try of blinking withoutd having to control timings in main loop
class Blinker
{
  public:
    explicit Blinker(unsigned long interval):
    mInterval(interval)
    {
      //pinMode(FLOWMETER_MARK_PIN, OUTPUT);
      //digitalWrite(FLOWMETER_MARK_PIN, LOW);
    }

    void loop(bool on)
    {
      if(on)
      {
        if(mInterval == 0)
        {
          digitalWrite(FLOWMETER_MARK_PIN, HIGH);
          return;
        }

        unsigned long now = millis();
        if(now - mLast > mInterval)
        {
          off = !off;
          digitalWrite(FLOWMETER_MARK_PIN, off ? LOW : HIGH);
          mLast = now;
        }
      }
      else
      {
        digitalWrite(FLOWMETER_MARK_PIN, LOW);
      } 
    }

    void changeInterval(unsigned long interval)
    {   
      mInterval = interval;      
    }

  protected:
    unsigned long mInterval = 0UL;
    unsigned long mLast = 0UL;
    bool off = true;
};

// Configuration variables for flowmeter
const int Qmax = 1;      // Liters per minute max to consider a leak
const int Tmax = 20;      // Minutes max with open valve
const int Tcheck = 120;   // Minutes to wait ext check. 
const int Tverif = 6;    // Seconds to have open valve to verify leaks

// State variables
enum State {
    WAITING_FOR_IRRIGATION,
    IRRIGATION_ACTIVE,
    PIPE_FAILURE,
    VERIFYING_LEAK
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

// Blinker (does not work)
Blinker blinker(0);

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
            
            blinker.loop(false);
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
            
            blinker.loop(false);
            break;

        case PIPE_FAILURE:
            Serial.println("Leak detected. Valve closed.");

            blinker.loop(true);
            blinker.changeInterval(0);
            
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
          
            blinker.changeInterval(500);
            blinker.loop(true);       

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
      out["Hc"] = humCable; // From sensor, we are sending decimal digits in this field
      if(tempCable != -1)
      {
        tempPrecise = tempCable + ((float)humCable/100); // TODO
        out["Tp"] = tempPrecise;
      }
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

////////////////////////////////////////
// Some methods to unpack 32bit RF data
////////////////////////////////////////

unsigned long field_mask(int field)
{
   unsigned long mask = ( 0x7F << field*7);   
   return mask;
}
 
unsigned long get_sign(unsigned long v,int field,bool sig)
{
    unsigned long mask = field_mask(field);
    unsigned long smask = 0;

    if(sig)
    {
      smask = (0b1000000 << field*7);
    
      int sign = (v & mask) & smask;
    
      return sign>0?-1:1;
    }
    else
      return 1;
}

unsigned long get_val(unsigned long v,int field,bool sig)
{
    unsigned long vmask = 0;
    int sign = 1;
    
    if(sig)
    {
      // we must take into account sign bit
      sign = get_sign(v,field,sig);
      vmask = (0b0111111 << field*7);
    }
    else
    {
       vmask = (0b1111111 << field*7);
    }

    long x = (v & vmask) >> field*7;
    
    return sign*x;
}

bool check_header(uint8_t head,unsigned long v)
{
    unsigned long hmask = (0b1111 << 4*7);

    uint8_t x = ((v & hmask) >> 4*7 ) & 0xFF;
    
    return (x == head);
}

void loop()
{
  uint32_t rfValue = 0;
  
  unsigned long now = millis();

  thing.handle();

  digitalWrite (RF433_RX_MARK_PIN, HIGH); 
   
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
    Serial.println(rfValue);

    // check packet header
    //      
    if(check_header(0xA,rfValue))
    {
      lastRfValue = rfValue;
      
      Serial.println("HEADER 0xA ");
      humCable = get_val(rfValue,0,false);
      humSonda = get_val(rfValue,1,false);
      tempCable = get_val(rfValue,2,true);
      tempSonda = get_val(rfValue,3,true);
 
      Serial.print("humCable: ");
      Serial.println(humCable);
      Serial.print("humSonda: ");
      Serial.println(humSonda);
      Serial.print("tempSonda: ");
      Serial.println(tempSonda);
      Serial.print("tempCable: ");
      Serial.println(tempCable);
      
      lastReception = now;
    }
    else if(check_header(0xB,rfValue))
    {
       lastRfValue = rfValue;

       Serial.println("HEADER 0xB ");
      //        voltage  pressure   state   unused
      // 1011 |  13bit  | 7bit    | 2bit  | 000
      // Pressure is relative to 1000

      // Take first bits 0b00011000 = 0x18 and shift to remove unused
      uint8_t state = (rfValue & (0x18)) >> 3;
      powerState = state;
      
      // Use uint16_t to have enough space to store the bits
      uint16_t pres = (rfValue & (uint16_t)(0b111111100000))>>5 ;
      // Once we have the 7bit,reuse the function get_val() to extract sign and value
      presSonda = 1000+get_val(pres,0,true);
      
      // Remove header and 12bits
      voltage = (rfValue & 0x0FFFF000) >> 12;
      
      Serial.print("state: ");
      Serial.println(powerState);
      Serial.print("Pressure: ");
      Serial.println(presSonda);
      Serial.print("Voltage: ");
      Serial.println(voltage);
      
      lastReception = now;
    }
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
      }
    }
    else
    {
      // Nothing received.
      lastRf = (now - lastReception)/1000;
      lastRf2 = (now - lastReception2)/1000;
      
      return;
    }
    
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

