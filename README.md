# orchard-sensors
An Arduino project about using couple of sensors to measure temperature and humidity and send data via RF433 to an ESP (E12) with internet connection, and control a fow of water.
This allows to have the sensors up to 150m away (orchard or garden) of the point where ESP can stay connected to internet (house), to send data to a ThingerIO server. 

The pair of sensors can be:
- A couple of Bosch BME-280
- A BME-280 and a thermistor. (thermistor for better accuracy)

The ESP also reads a water flow counter and a solenoid valve to shut off the water flow if some given amount of time has elapsed. 
The motivation, is to detect if there is some problem in the main pipe that feeds a couple of automatic water irrigation timers, so if they got stuck or the pipe is broken the water flow can be closed without human intervention and prevent surprises in water fee.
Water devices are:
- A solenoide valve revamped from an old water irrigation timer
- A HAll effect water flowmeter, that provides pulses that are counted by ESP as interrupts per time interval.
