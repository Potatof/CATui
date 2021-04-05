//    Pin 	Function 	                    ESP-8285 Pin
//    TX 	  TXD 	                        TXD
//    RX 	  RXD 	                        RXD
//    A0 	  Analog input, max 3.2V 	      A0
//    D0 	  IO 	                          GPIO16
//    D1 	  IO, SCL 	                    GPIO5
//    D2 	  IO, SDA 	                    GPIO4
//    D3 	  IO, 10k Pull-up 	            GPIO0
//    D4 	  IO, 10k Pull-up, BUILTIN_LED 	GPIO2
//    D5 	  IO, SCK 	                    GPIO14
//    D6 	  IO, MISO 	                    GPIO12
//    D7 	  IO, MOSI 	                    GPIO13
//    D8 	  IO, 10k Pull-down, SS 	      GPIO15
//    G 	  Ground 	                      GND
//    5V 	  5V 	                          -
//    3V3 	3.3V 	                        3.3V
//    RST 	Reset 	                      RST

#include <Arduino.h>
#include <Ticker.h>

#define LED 2 //On board LED

ICACHE_RAM_ATTR void askForFrequency();
ICACHE_RAM_ATTR void updateEncoder();
ICACHE_RAM_ATTR void updateFrequency();

void initComm();
void initGpio();
void initTimer();
void askForFrequency();
void setVfoStep(int modumode, int sentFrequency);
void updateFreqency(int refFrequency, bool up);
void sendFrequency();
void receiveFrequency();

const int P1 = 5; //    D1 	  IO, SCL    GPIO5
const int P2 = 4; //    D2 	  IO, SDA    GPIO4
const int BAUDRATE = 115200;
const int SERIAL_TIMEOUT = 2000;
const int FREQ_QUERY_RATE = 300;
const int FREQ_SEND_RATE = 100;
const int DEFAULT_FREQ = 127000000;
const int AIR_SUB_CHANNEL = 8333;
const int AIR_CHANNEL = 25000;
const int DEFAULT_VFO_STEP = 1000;
const int ROTARY_STEP = 100;

const int LOW_AIR_BAND = 118000000;
const int HIGH_AIR_BAND = 140000000;
const int LOW_FM_BAND = 88000000;
const int HIGH_FM_BAND = 108000000;

volatile int lastEncoded = 0;
volatile int frequency = DEFAULT_FREQ;
volatile int readFrequency = DEFAULT_FREQ;
volatile int vfoStep = DEFAULT_VFO_STEP;
volatile int subFreq = 0;
volatile int modumode = 5; // AM
volatile bool asked = false;
volatile bool asktog = false;
volatile bool locked = false;

Ticker readFrequencyTicker;

/////////////////////////////////////////////////////////////////////////////////////////
/// setup                                                                             ///
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  initComm();
  initGpio();
  initTimer();
}

void initComm()
{
  Serial.setDebugOutput(false);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.clearWriteError();
  Serial.begin(BAUDRATE, SERIAL_8N1);
  while (!Serial)
  {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }
}

void initGpio()
{
  pinMode(LED, OUTPUT);
  //digitalWrite(P1, HIGH); //turn pullup resistor on
  //digitalWrite(P2, HIGH); //turn pullup resistor on
  pinMode(P1, INPUT);
  pinMode(P2, INPUT);
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
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Timer management                                                                  ///
/////////////////////////////////////////////////////////////////////////////////////////
void askForFrequency()
{
  if (!asked && !locked)
  {
    asked = true;
    digitalWrite(LED, !(digitalRead(LED))); //Toggle LED Pin
    if (!asktog)
    {
      Serial.flush();        // wait until TX buffer is empty
      Serial.println("FA;"); // ask for current frequency
    }
    else
    {
      Serial.flush();        // wait until TX buffer is empty
      Serial.println("MD;"); // ask for modulation mode
    }
    asktog = !asktog;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
/// Serial comm management                                                            ///
/////////////////////////////////////////////////////////////////////////////////////////
void receiveFrequency()
{
  if (asked && !locked)
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

void sendFrequency()
{
  digitalWrite(LED, !(digitalRead(LED))); //Toggle LED Pin
  char result[11];
  sprintf(result, "%011d", frequency);
  Serial.flush(); // wait until TX buffer is empty
  Serial.println("FA" + String(result) + ";");
  delay(20);
}
