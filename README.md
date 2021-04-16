# cathui
Hardware user interface to control SDR software like 'SDR Console' using CAT protocol with ESP8266 D1 mini lite card (or other) and HN3806-AB-600N rotary encoder
Although not using WiFi, I used an ESP8266 D1 mini lite, because it was inexpensive, had a serial interface and was easy to set up.

Simple Prototype:
![alt text](https://github.com/Potatof/cathui/blob/master/docs/proto.jpg)
The rotary encoder has an independent power supply (7V) because it does not work with USB 5V vcc.
Two pull-up resistors have been added from the VCC to the GPIO being used.
It works fine with SDR Console software.

