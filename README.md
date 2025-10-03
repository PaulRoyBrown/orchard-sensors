# orchard-sensors
An Arduino project about using a couple of sensors attached to an **Arduino Mini Pro** to measure temperature and humidity and send data via RF433 to an **ESP8266 (E12)** with internet connection, and control a flow of water.
This allows to have the sensors up to 150m away (orchard or garden) of the point where ESP8266 server can stay connected to internet (house), to send data to a ThingerIO **public** site, where data is graphed at one sample per minute.

The pair of sensors controller by the Arduino Mini Pro can be:
- A couple of Bosch **BME-280**
- A BME-280 and a **thermistor**. (calibrated hermistor for better accuracy)
Sensors board is powered by a solar panel (2.5W 5V/500mAh) that feeds a LiPo battery, and load during day and discharge duriong nigh is controlled by a MCP78371 chip.

The ESP also reads a water flow counter and a solenoid valve to shut off the water flow if some given amount of time has elapsed. 
The motivation, is to detect if there is some problem in the main pipe that feeds a couple of automatic water irrigation timers, so if they got stuck or the pipe is broken the water flow can be closed without human intervention and prevent surprises in water fee.
Water devices are:
- A solenoid valve revamped from an old water irrigation timer,
- A DC dual H-Bridge motor controller driver board, based on MX1616 controller, to command the valve.
- A Hall effect water flowmeter, that provides pulses that are counted by ESP as interrupts per time interval.
