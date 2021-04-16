# cathui
A hardware user interface to control CAT compatible radio software such as 'SDR Console'. Consisting of a Wemos D1 mini lite ESP8266 card and a HN3806-AB-600N rotary encoder. Although not using WiFi I used an ESP8266 D1 mini lite because it was inexpensive, it has a serial interface and is easy to set up.

Simple Prototype:
![alt text](https://github.com/Potatof/cathui/blob/master/docs/proto.jpg)
The rotary encoder has an independent power supply (7V) because it does not work with USB 5V vcc.
Two pull-up resistors have been added from the VCC to the GPIO being used.
It works fine at 57600 bds with SDR Console software.

