# orchard-sensors
An Arduino project about using a couple of sensors attached to an **Arduino Mini Pro** to measure temperature and humidity in my orchard and send data via RF433 to an **ESP8266 (E12)** that is connected to domestic WiFi and also controls flow of water in an irrigation pipe.
This allows to have the sensors up to 150m away (orchard or garden) of the point where ESP8266 server can stay connected to internet (house), to send data to a [ThingerIO](https://docs.thinger.io/quick-start) **public** site, where data is [graphed](https://console.thinger.io/dashboards/THP?authorization=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiJEYXNoYm9hcmRfVEhQIiwic3ZyIjoiZXUtY2VudHJhbC5hd3MudGhpbmdlci5pbyIsInVzciI6IkJvbGkifQ.M497lwl8ptuKFHAIZS7OxW-C9J2TpUQ5bXjZf_JB_Zo) at one sample per minute.

<img width="540" height="381" alt="image" src="https://github.com/user-attachments/assets/df27c676-24d6-482d-98f5-f5f88c70a33c" />


Beside the two sensors (red and blue lines in pictures) I have also added another Arduino mini pro that just uses a single sensor to send environment data from inside my house (green line in next picture). So in total,we receive RF messages from three sensors, two in orchard and one inside the house.

<img width="1570" height="731" alt="image" src="https://github.com/user-attachments/assets/43d3f4d0-9b45-4532-90d5-284b1128df6d" />

We explain now a little what each part does in the project.

<h2>Sensors Board (Mini Pro)</h2>
The pair of sensors controller by the Arduino Mini Pro can be:

- A couple of Bosch **BME-280**
- A BME-280 and a **thermistor** (calibrated thermistor for better accuracy) with an **ADS1115** ADC chip to measure thermistor voltage.
 
With two sensors, we can measure air and soil temperature. I'm now measuring soil temperature at 60cm below ground. (Why?...Because of curiosity....What's the phase delay between a peak in temperature at ground level and 60cm below it?.Will average temperature at that depth slowly increase due to global warming?...not in the three years I'm been watching...)

Sensors board is powered by a solar panel (2.5W 5V/500mAh) that feeds a LiPo battery, and loads during day and discharges during night. This is controlled by a **MCP78371** chip.
Being powered by battery, is important to monitor battery voltage, so sensors board in the orchard has a power saving mechanism based on its own voltage monitoring. If voltage drops below some levels, it will transmit data less often to save battery charge.  

Battery capacity is 150mA or 350mA and sensor data is sent via RF in 433Mhz band. During day, measurements are sent each minute. During night, in winter, voltage monitoring sensor board enters in power save mode sending messages each 3 min. 

This is enough to keep this level all night with no problem (and even some days if no solar power is present). In case voltage decreases even more (very unusual), messages are sent each 10min, and even each 30min. This state can be mantained for much more than a week.

Measured current consuptiom in sleep state is about 180uA for the two BME280 sensors board version. For thermistor version, raises to 700uA (and i'm trying to reduce that).When board wakes up each minute and measures and sends RF data, current jumps to 8mA for 5 seconds or so. Thermistor constant current used to do accurate measurments is produced by a typical **LM334** current source, soon to be replaced for the **REF200U** chip, with lower temperature drift.

<h2>Server board (ESP8266-E12)</h2>
My ESP server also reads a water flow counter and a solenoid valve to shut off the water flow if some given amount of time has elapsed. 
The motivation, is to detect if there is some problem in the main pipe that feeds a couple of automatic water irrigation timers placed in the orchard, so if they got stuck or the pipe is broken the water flow can be closed without human intervention and prevent surprises in monthly water fee. (Pipe broke once, and water was flowing all night long...) 

Water control devices are:
- A solenoid valve revamped from an old water irrigation timer, like [this](https://www.amazon.es/Temporizador-Automatico-Acoplamiento-Aplicables-Programador/dp/B07WSWNPPH?pd_rd_w=h13JV&content-id=amzn1.sym.20817a10-d709-4b1f-b971-9466d7e8ae8d&pf_rd_p=20817a10-d709-4b1f-b971-9466d7e8ae8d&pf_rd_r=GN6S7M7MD3MNNQ2PWESW&pd_rd_wg=ykqpR&pd_rd_r=00232e09-464d-4d5b-b19d-6409ca1d73be&ref_=pd_bap_d_grid_rp_csi_pd_ys_c_rfy_rp_crs_0_t&th=1)
- A DC dual H-Bridge motor controller driver board, based on **MX1616** controller, to command the valve.
- A Hall effect water [flowmeter](https://www.amazon.es/SWAWIS-Piezas-Sensor-Medidor-Interruptor/dp/B0C2GT6LHY?pd_rd_w=LfQrS&content-id=amzn1.sym.3c8e6299-45ee-4267-8322-a7dde32d8230&pf_rd_p=3c8e6299-45ee-4267-8322-a7dde32d8230&pf_rd_r=DDT2R3GDZPC3GS7S6BM5&pd_rd_wg=j0VtN&pd_rd_r=b3262f16-20b0-4701-b609-dec38e9eb10a&psc=1&ref_=pd_bap_d_grid_rp_csi_prsubs_0_t), that provides pulses that are counted by ESP as interrupts per time interval.

The ESP server receives RF data using typical superheterodyne **SRX882** chip. Each RF sensor sends messages with an identifier, and using that he knows how to parse its data (packed as a 32bit long type) and its origin (house or orchard). 

Server box is placed outside, up in one wall of my house where WiFi access is feasible, so it can open a session in ThingerIO cloud. It's inside a plastic box and feeded by a little 220V to 3.3V converter. Is attached to a six wire cable that receives the cables for solenoid valve and waterflow meter (red lines in picture above). 
