# orchard-sensors
An Arduino project about using couple of sensors to measure temperature and humidity and send data via RF433 to an ESP (E12) with internet connection. 
This allows to have the sensors up to 150m away (orchard or garden) of the point where ESP can stay connected to internet, to send data to a thingerIO server. 

The sensors can be:
- A couple of Bosh BME-280
- A BME-280 and a thermistor

The ESP also reads a water flow counter and a solenoid valve to shut off the water flow if some given amount of time has elapsed. 
The motivation, is to detect if there is some problem in the main pipe that feeds a couple of automatic water irrigation timers, so if they got stuck or the pipe is broken the water flow can be closed without human intervention.
Water devide are:
- A solenoide valve revamped from and old water irrigation timer
- A water flowmeter based on Hall effect, that provides pulses that are counted by ESP as interrupts per time.
