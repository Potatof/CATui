# cathui
Hardware user interface to control SDR software like 'SDR Console' using CAT protocol with ESP8266 D1 mini lite card (or other) and HN3806-AB-600N rotary encoder

It works fine with SDR Console software

Simple Prototype:
![alt text](https://github.com/Potatof/cathui/blob/master/docs/proto.jpg)
The rotary encoder has an independent power supply (7V) because it does not work with USB 5V vcc.
Two pull-up resistors have been added from the VCC to the GPIO being used.
