//
// ---------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------
//  ESP8266 best pins to use:
//    Pin 	Function 	                    ESP-8285 Pin      Notes
//    TX 	  TXD 	                        TXD, GPIO1        HIGH at boot - debug output at boot, boot fails if pulled LOW
//    RX 	  RXD 	                        RXD, GPIO3        HIGH at boot
//    A0 	  Analog input, max 3.2V 	      A0                Analog Input
//    D0 	  IO 	                          GPIO16            HIGH at boot - used to wake up from deep sleep
//    D1 	  IO, SCL 	                    GPIO5             Often used as SCL (I2C)
//    D2 	  IO, SDA 	                    GPIO4             Often used as SDA (I2C)
//    D3 	  IO, 10k Pull-up 	            GPIO0             Connected to FLASH button, boot fails if pulled LOW
//    D4 	  IO, 10k Pull-up, BUILTIN_LED 	GPIO2             HIGH at boot - connected to on - board LED, boot fails if pulled LOW
//    D5 	  IO, SCK 	                    GPIO14            SPI (SCLK)
//    D6 	  IO, MISO 	                    GPIO12            SPI (MISO)
//    D7 	  IO, MOSI 	                    GPIO13            SPI (MOSI)
//    D8 	  IO, 10k Pull-down, SS 	      GPIO15            Pulled to GND - SPI(CS) Boot fails if pulled HIGH
//    G 	  Ground 	                      GND
//    5V 	  5V 	                          -
//    3V3 	3.3V 	                        3.3V
//    RST 	Reset 	                      RST
// ---------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------
//

#include <Arduino.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>

//#define USE_PULLED_UP_IO // use pulluped IO

ICACHE_RAM_ATTR void askForFrequency();
ICACHE_RAM_ATTR void updateEncoder();
ICACHE_RAM_ATTR void updateFrequency();

void askForFrequency();
void initComm();
void initGpio();
void initTimer();
void initWifi();
void receiveFrequency();
void sendFrequency();
void serialRxFlush();
void serialTxFlush(String command);
void setVfoStep(int modumode, int sentFrequency);
void updateFreqency(int refFrequency, bool up);

#ifdef USE_PULLED_UP_IO
const int P1 = 0; //    D3 	  IO, FLASH   GPIO0 -> Pulled Up but no more LED...
const int P2 = 2; //    D4 	  IO, BLUELED GPIO2 -> Pulled Up but boot can crash...
#else
#define LED 2 //On board LED
const int P1 = 5; //    D1 	  IO, SCL     GPIO5 -> pullup resistor to vcc needed
const int P2 = 4; //    D2 	  IO, SDA     GPIO4 -> pullup resistor to vcc needed
#endif

const int AIR_CHANNEL = 25000;
const int AIR_SUB_CHANNEL = 8333;
const int BAUDRATE = 57600;
const int DEFAULT_FREQ = 127000000;
const int DEFAULT_VFO_STEP = 1000;
const int FREQ_QUERY_RATE = 300;
const int FREQ_SEND_RATE = 100;
const int HIGH_AIR_BAND = 140000000;
const int HIGH_FM_BAND = 108000000;
const int LOW_AIR_BAND = 118000000;
const int LOW_FM_BAND = 88000000;
const int ROTARY_STEP = 100;
const int SERIAL_TIMEOUT = 2000;

volatile bool asked = false;
volatile bool asktog = false;
volatile bool locked = false;
volatile int frequency = DEFAULT_FREQ;
volatile int lastEncoded = 0;
volatile int modumode = 5; // AM
volatile int readFrequency = DEFAULT_FREQ;
volatile int subFreq = 0;
volatile int vfoStep = DEFAULT_VFO_STEP;

Ticker readFrequencyTicker;

/////////////////////////////////////////////////////////////////////////////////////////
/// setup                                                                             ///
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  initWifi();
  initComm();
  initGpio();
  initTimer();
}

void initWifi()
{
  WiFi.softAPdisconnect(true); // Disable default Wifi Access Point (something like 'FaryLink_xxxxxx')
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}

void initComm()
{
  Serial.setDebugOutput(false);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.clearWriteError();
  Serial.begin(BAUDRATE, SERIAL_8N1);
  while (!Serial)
  {
    yield();
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }
}

void initGpio()
{
#ifndef USE_PULLED_UP_IO
  pinMode(LED, OUTPUT);
#endif
  //digitalWrite(P1, HIGH); //turn pullup resistor on
  //digitalWrite(P2, HIGH); //turn pullup resistor on
  pinMode(P1, INPUT_PULLUP);
  pinMode(P2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(P1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P2), updateEncoder, CHANGE);
}

void initTimer()
{
  readFrequencyTicker.attach_ms(FREQ_QUERY_RATE, askForFrequency);
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Main loop                                                                         ///
/////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  receiveFrequency();
  yield(); // Do the rest
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Timer management                                                                  ///
/////////////////////////////////////////////////////////////////////////////////////////
void askForFrequency()
{
  if (!asked && !locked)
  {
    asked = true;
#ifndef USE_PULLED_UP_IO
    digitalWrite(LED, !(digitalRead(LED))); //Toggle LED Pin
#endif
    if (!asktog)
    {
      serialTxFlush("FA;"); // ask for current frequency
    }
    else
    {
      serialTxFlush("MD;"); // ask for modulation mode
    }
    asktog = !asktog;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Serial comm management                                                            ///
/////////////////////////////////////////////////////////////////////////////////////////
void receiveFrequency()
{
  if (locked)
  {
    serialRxFlush(); // wait until RX buffer is empty
  }
  else if (asked)
  {
    String rxresponse = Serial.readStringUntil(';');
    if (rxresponse.startsWith("FA"))
    {
      // FAP1(11)
      // FA00127045371;
      // FA
      // 00127045371    Specify the frequency in Hz
      // IF
      // 00127045371    Specify the frequency in Hz
      // 0000           Frequency step size
      // +00000         RIT/ XIT frequency ▒99999 in Hz
      // 0              0: RIT OFF, 1: RIT ON
      // 0              0: XIT OFF, 1: XIT ON
      // 0              0: RX, 1: TX
      // 0              Operating mode
      // 0              See FR and FT commands.
      // 5              Scan status. See SC command.
      // 0              Split operation status. See SP command.
      // 0              0: OFF, 1: TONE, 2: CTCSS, 3: DCS
      // 0000           Tone frequency. See TN command.
      // 0              Shift status. See OS command.
      readFrequency = rxresponse.substring(2, 13).toInt();
    }
    else if (rxresponse.startsWith("MD"))
    {
      // 0: None
      // 1: LSB
      // 2: USB
      // 3: CW
      // 4: FM
      // 5: AM
      // 6: FSK
      // 7: CWR (CW Reverse)
      // 8: Tune
      // 9: FSR (FSK Reverse)
      modumode = rxresponse.substring(2, 3).toInt();
    }
    setVfoStep(modumode, frequency);
    asked = false; // On timeout also
  }
}

void sendFrequency()
{
  serialRxFlush(); // wait until RX buffer is empty
#ifndef USE_PULLED_UP_IO
  digitalWrite(LED, !(digitalRead(LED))); //Toggle LED Pin
#endif
  char result[11];
  sprintf(result, "%011d", frequency);
  serialTxFlush("FA" + String(result) + ";;"); // Send new frequency
}

void serialTxFlush(String command)
{
  Serial.flush(); // wait until TX buffer is empty
  delay(5);
  Serial.println(command);
  delay(5);
}

void serialRxFlush()
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

void setVfoStep(int modumode, int frequency)
{
  switch (modumode)
  {
  case 0:
    vfoStep = 1000;
    break;
  case 1:
  case 2:
  case 3:
  case 7: // USB LSB CW CWR
    vfoStep = 200;
    break;
  case 4: // FM
    if ((frequency >= LOW_FM_BAND) && (frequency <= HIGH_FM_BAND))
    {
      vfoStep = 25000;
    }
    else
    {
      vfoStep = 500;
    }
    break;
  case 5: // AM
    if ((frequency >= LOW_AIR_BAND) && (frequency <= HIGH_AIR_BAND))
    {
      vfoStep = 5000;
    }
    else
    {
      vfoStep = 500;
    }
    break;
  case 6:
  case 9: // FSK FSKR
    vfoStep = 500;
    break;
  case 8: // TUNE
    vfoStep = 5000;
    break;
  default:
    vfoStep = 200;
    break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
/// rotary encoder management                                                         ///
/////////////////////////////////////////////////////////////////////////////////////////
void updateEncoder()
{
  if (!locked)
  {
    locked = true;
    int MSB = digitalRead(P1); //MSB = most significant bit
    int LSB = digitalRead(P2); //LSB = least significant bit

    int encoded = (MSB << 1) | LSB;         //converting the 2 pin value to single number
    int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
    lastEncoded = encoded;                  //store this value for next time
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
      subFreq += 1;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
      subFreq -= 1;

    if (subFreq > FREQ_QUERY_RATE)
    {
      subFreq = 0;
      updateFreqency(readFrequency, true);
    }
    else if (subFreq < -FREQ_QUERY_RATE)
    {
      subFreq = 0;
      updateFreqency(readFrequency, false);
    }
    locked = false;
  }
}

void updateFreqency(int refFrequency, bool up)
{
  int vfoSubStep = vfoStep;
  int channel = int(round((double(refFrequency) - double(LOW_AIR_BAND)) / double(AIR_CHANNEL)));
  int cfreq = LOW_AIR_BAND + (channel * AIR_CHANNEL);
  int deltaFreq = refFrequency - cfreq;

  if ((refFrequency >= LOW_AIR_BAND) && (refFrequency <= HIGH_AIR_BAND))
  { // Air Band ?
    if (abs(deltaFreq) < (AIR_SUB_CHANNEL * 2))
    {
      vfoSubStep = 5000 / 3;
    }
  }

  int roundedStep = (deltaFreq / vfoSubStep) * vfoSubStep;
  if (up)
  {
    frequency = cfreq + roundedStep + vfoSubStep;
  }
  else
  {
    frequency = cfreq + roundedStep - vfoSubStep;
  }
  sendFrequency();
  readFrequency = frequency;
}
