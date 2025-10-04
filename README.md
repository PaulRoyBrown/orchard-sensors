# orchard-sensors
An Arduino project about using a couple of sensors attached to an **Arduino Mini Pro** to measure temperature and humidity and send data via RF433 to an **ESP8266 (E12)** that is connected to domestic WiFi and also controls flow of water in an irrigation pipe.
This allows to have the sensors up to 150m away (orchard or garden) of the point where ESP8266 server can stay connected to internet (house), to send data to a ThingerIO **public** site, where data is graphed at one sample per minute.

Beside the two sensors (red and blue lines) I have also added another Arduino mini pro that just uses a single sensor to send environment data from inside my house (green line in next picture). )So in total,we receive RF messages from three sensors)

<img width="1570" height="731" alt="image" src="https://github.com/user-attachments/assets/43d3f4d0-9b45-4532-90d5-284b1128df6d" />

We explain now a little what each part does in the project.

<h2>Sensors Board (Mini Pro)</h2>
The pair of sensors controller by the Arduino Mini Pro can be:

- A couple of Bosch **BME-280**
- A BME-280 and a **thermistor** (calibrated thermistor for better accuracy) with an **ADS1115** ADC chip to measure thermistor voltage.
 
With two sensor, we can measure air and soil temperature. I'm now measuring soil temperature at 60cm below ground. (Why?...Because of curiosity....Will average temperature at that depth slowly increase due to global warming?...not in the three years I'm been watching...)

Sensors board is powered by a solar panel (2.5W 5V/500mAh) that feeds a LiPo battery, and loads during day and discharges during night. This is controlled by a **MCP78371** chip.
Being powered by battery, is important to monitor battery voltage, so sensors board have a power saving mechanism based on its own voltage monitoring. 

Battery capacity is 150mA or 350mA and sensor data is sent via RF in 4333Mhz band. During day, measurements are sent each minute. During night, in winter, voltage monitoring sensor board enters in power save mode sending messages each 3 min. 

This is enough to keep this level all night with no problem (and even some days if no solar power is present). In case voltage decreases even more (very unusual), messages are sent each 10min, and even each 30min. This state can be mantained for much more than a week.

Measured current consuptiom in sleep state is about 180uA for the two BME280 sensors board version. For thermistor version, raises to 700uA (and i'm trying to reduce that).When board wakes up each minute and measures and sends RF data, current jumps to 8mA for 5 seconds or so.

<h2>Server board (ESP8266-E12)</h2>
Server is placed outside, up in one wall of my house where WiFi access is feasible, so it can open a session in ThingerIO cloud. It's inside a plastic box and feeded by a little 220V to 3.3V converter. Is attached to a six wire cable that receives the cables for solenoid valve and waterflow meter. 

My ESP server also reads a water flow counter and a solenoid valve to shut off the water flow if some given amount of time has elapsed. 
The motivation, is to detect if there is some problem in the main pipe that feeds a couple of automatic water irrigation timers, so if they got stuck or the pipe is broken the water flow can be closed without human intervention and prevent surprises in monthly water fee.

Water control devices are:
- A solenoid valve revamped from an old water irrigation timer,
- A DC dual H-Bridge motor controller driver board, based on **MX1616** controller, to command the valve.
- A Hall effect water flowmeter, that provides pulses that are counted by ESP as interrupts per time interval.

The ESP server receives RF data using typical superheterodyne **SRX882** chip. Each RF sensor sends messages with an identifier, and using that he knows how to parse its data (packed as a 32bit long type) and its origin (house or orchard). 

