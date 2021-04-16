# cathui
Hardware user interface to control SDR software like 'SDR Console' using CAT protocol with ESP8266 D1 mini lite card (or other) and HN3806-AB-600N rotary encoder

Simple Prototype:
![alt text](https://github.com/Potatof/cathui/master/docs/proto.jpg?raw=true)
The rotary encoder has an independent power supply (7V) because it does not work with USB vcc.
Two pull-up resistors have been added from the VCC to the GPIO being used.
