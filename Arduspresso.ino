#include <TimedAction.h>
#include <LiquidCrystal.h>
#include <NewPing.h>
#include <EmonLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

char ver[] = "v1.9";

//Ultrasonic sensor
#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 32 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
unsigned int distance;
unsigned int watercontainerMaximumMeasurement = 7 + 18; //7 is maxed up in the tank and 18 cm down is when the tank is empty
unsigned int watercontainerMinimumMeasurement = 7; //when the tank is maxed up the ultrasonic sensor has 7cm to the water level
int waterLevelPercent;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//LCD display
/*
 LCD-screen circuit
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD V0/3 pin to pin 6 (contrast)
 * ends to +5V and ground
 */
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int contrastPin = 6; // PWM pin (contrast level)
static byte droplet_glyph[] = {  B00100,  B00100,  B01110,  B01110,  B11111,  B11111,  B11111,  B01110 };
byte glyphs[5][8] = {  { B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000 },
  { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111 },
  { B11111, B11111, B00000, B00000, B00000, B00000, B11111, B11111 },
  { B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111 } ,
  { B00000, B00000, B00000, B00000, B00000, B01110, B01110, B01110 }
};

// the width in characters of a big digit
// (excludes space between characters)
const int digitWidth = 3;
//arrays  to index into custom characters that will comprise the big numbers
const char bigDigitsTop[10][digitWidth] = { 3, 0, 3, 0, 3, 32, 2, 2, 3, 0, 2, 3, 3, 1, 3, 3, 2, 2, 3, 2, 2, 0, 0, 3,   3, 2, 3, 3, 2, 3};
const char bigDigitsBot[10][ digitWidth] = { 3, 1, 3, 1, 3, 1,  3, 1, 1, 1, 1, 3, 32, 32, 3, 1, 1, 3, 3, 1, 3, 32, 32, 3, 3, 1, 3, 1, 1, 3};
char buffer[12]; // used to convert a number into a string
int screenPowerPin = 10;
int screenTimerSeconds = 5; //this is the time to countdown inorder to keep the screen active
boolean writeDroplet = true;

//brewtimer
int secondsBrewed = 1;
bool pumpIsOn = false;
EnergyMonitor brewMonitor; // with 18ohms burden
int brewMonitorPin = 1;
double pumpIrms;

//boilerStatus
EnergyMonitor boilerMonitor; // with 18ohms burden
int boilerMonitorPin = 2;
double boilerIrms;

//boilerTemp
int oneWirePin = 7;
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);
float boilerTemperature;

//433Mhz communication
//thank you http://www.pojpoj.se/klipparever1.html for the RF code
const byte TX_PIN = 13;

const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME * 2;

#define SEND_HIGH() digitalWrite(TX_PIN, HIGH)
#define SEND_LOW() digitalWrite(TX_PIN, LOW)

// Buffer for Oregon message
byte OregonMessageBuffer[9];

/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void sendZero(void)
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}

/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void sendOne(void)
{
  SEND_LOW();
  delayMicroseconds(TIME);
  SEND_HIGH();
  delayMicroseconds(TWOTIME);
  SEND_LOW();
  delayMicroseconds(TIME);
}

/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/

/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}

/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for (byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
  sendPreamble();
  sendData(data, size);
  sendPostamble();
}

/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[] = {0xFF, 0xFF};
  sendData(PREAMBLE, 2);
}

/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
  byte POSTAMBLE[] = {0x00};
  sendData(POSTAMBLE, 1);
}

/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}

/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type)
{
  data[0] = type[0];
  data[1] = type[1];
}

/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel)
{
  data[2] = channel;
}

/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID)
{
  data[3] = ID;
}

/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if (!level) data[4] = 0x0C;
  else data[4] = 0x00;
}

/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp)
{
  // Set temperature sign
  if (temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;
  }
  else
  {
    data[6] = 0x00;
  }

  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt / 10 - (float)td) * 10);

  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;

  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
  data[7] = (hum / 10);
  data[6] |= (hum - data[7] * 10) << 4;
}

/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;

  for (byte i = 0; i < count; i++)
  {
    s += (data[i] & 0xF0) >> 4;
    s += (data[i] & 0xF);
  }

  if (int(count) != count)
    s += (data[count] & 0xF0) >> 4;

  return s;
}

/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
  data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}

//action
TimedAction printLCDDataAction = TimedAction(400, printLCDData);
TimedAction ultrasonicSensorAction = TimedAction(5000, runUltrasonicSensor);
TimedAction energyMonitorAction = TimedAction(300, monitorEnergy);
TimedAction brewTimerAction = TimedAction(1000, brewTimer);
TimedAction displayTimerAction = TimedAction(1000, displayTimerCountdown);
TimedAction boilerMonitorAction = TimedAction(1500, checkBoiler);
TimedAction brewMonitorAction = TimedAction(1500, checkPump);
TimedAction send433Action = TimedAction(20000, send433);

void setup() {
  setupLCD();
  Serial.begin(9600);
  sensors.begin();

  brewMonitor.current(brewMonitorPin, 111.11);
  boilerMonitor.current(boilerMonitorPin, 111.11);

  pinMode(screenPowerPin, OUTPUT);
  runUltrasonicSensor(); //lets get an inital waterlevel reading

  pinMode(TX_PIN, OUTPUT);
  SEND_LOW();

  // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
  byte ID[] = {0x1A, 0x2D};

  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
  setId(OregonMessageBuffer, 0xBB); //BB=187
}

void setupLCD()
{
  // set up the LCD's number of columns and rows
  lcd.begin(20, 4);
  lcd.clear();
  delay(200);

  pinMode(contrastPin, OUTPUT); //contrast
  analogWrite(contrastPin, 30); // set the display contrast maximum 0 and minimum 255
  createNumberGlyphs();
  lcd.createChar(6, droplet_glyph);
  lcd.clear();
}

void createNumberGlyphs()
{
  for (int i = 0; i < 5; i++)
  {
    lcd.createChar(i, glyphs[i]);
  }
}

void showDigit(int digit, int position, int shiftdown)
{
  lcd.setCursor(position * (digitWidth + 1), 0 + shiftdown);
  for (int i = 0; i < digitWidth; i++)
    lcd.write(bigDigitsTop[digit][i]);
  lcd.setCursor(position * (digitWidth + 1), 1 + shiftdown);
  for (int i = 0; i < digitWidth; i++)
    lcd.write(bigDigitsBot[digit][i]);
}

void showNumber(int value, int position, int shiftdown) {
  int index; // index to the digit being printed, 0 is the leftmost digit
  itoa(value, buffer, 10); // see Recipe 2.8 for more on using itoa
  // display each digit in sequence
  for (index = 0; index < 10; index++)
  { char c = buffer[index];
    if ( c == 0) // check for null (not the same as '0')
      return; // the end of string character is a null, see Chapter 2
    c = c - 48; // convert ascii value to a numeric value  (see Recipe 2.9)
    showDigit(c, position + index, shiftdown);
  }
}

void loop() {

  energyMonitorAction.check();
  brewTimerAction.check();
  printLCDDataAction.check();
  displayTimerAction.check();
  ultrasonicSensorAction.check();
  boilerMonitorAction.check();
  brewMonitorAction.check();

  boilerTemperature = getTemp();

  send433Action.check();
}

void sendSensorData()
{
  Serial.print("BoilerMonitor Irms: ");
  Serial.println(boilerIrms);

  Serial.print("Pump Irms: ");
  Serial.println(pumpIrms);

  delay(200);
}

void checkPump()
{
  pumpIrms = brewMonitor.calcIrms(1480);
}

void checkBoiler()
{
  boilerIrms = boilerMonitor.calcIrms(1480);
}

void monitorEnergy()
{
  if (pumpIrms > 0.60 && boilerTemperature > 100)
  {
    pumpIsOn = true;
  }
  else
  {
    pumpIsOn = false;
  }

  //&& boilerTemperature > 50
  if (boilerIrms > 2)
  {
    if (boilerTemperature > 100)
    {
      screenTimerSeconds = 45; //gives the screen more seconds of activity
    }
    else
    {
      screenTimerSeconds = 20;
    }
  }
}

void printLCDData()
{
  if (secondsBrewed > 0)
  {
    lcd.clear();
    showNumber(secondsBrewed, 2, 1);
  }
  else
  {
    lcd.clear();
    showNumber(boilerTemperature, 0, 0);

    if (writeDroplet)
    {
      lcd.createChar(6, droplet_glyph);
      lcd.setCursor(0, 4);
      lcd.write(6);
    }

    writeDroplet = true;

    lcd.setCursor(1, 4);
    lcd.print(waterLevelPercent);
    lcd.write("%");
    lcd.setCursor(16, 4);
    lcd.write(ver);
  }
}
void displayTimerCountdown()
{
  if (screenTimerSeconds > 0)
  {
    screenTimerSeconds--;
  }

  if (screenTimerSeconds < 1)
  {
    digitalWrite(screenPowerPin, LOW);
  }
  else
  {
    digitalWrite(screenPowerPin, HIGH);
  }
}

void brewTimer()
{
  if (pumpIsOn)
  {
    secondsBrewed += 1;
  }
  else
  {
    secondsBrewed = 0;
  }

  if (secondsBrewed >= 99)
  {
    secondsBrewed = 1;
  }
}

void runUltrasonicSensor()
{
  unsigned int uS = sonar.ping();
  distance = uS / US_ROUNDTRIP_CM;

  if(uS < 1)
  {
    return;
  }
  
  if (distance <= watercontainerMinimumMeasurement)
  {
    waterLevelPercent = 100;
  }
  else
  {
    if (distance >= watercontainerMaximumMeasurement)
    {
      waterLevelPercent = 0;
    }
    else
    {

      waterLevelPercent = map(watercontainerMaximumMeasurement - (distance - watercontainerMinimumMeasurement), watercontainerMinimumMeasurement, watercontainerMaximumMeasurement, 0, 100);
    }
  }

  if (waterLevelPercent < 25)
  {
    writeDroplet = false;
  }
  else
  {
    writeDroplet = true;
  }
}

float getTemp() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void send433()
{
  sendToTellstick(boilerTemperature, waterLevelPercent, 0xBB);
}

void sendToTellstick(float temperature, byte humidity, byte Identitet)
{
  setId(OregonMessageBuffer, Identitet); //BB=187
  setBatteryLevel(OregonMessageBuffer, 1); // 0 : low, 1 : high
  setTemperature(OregonMessageBuffer, temperature); //org  setTemperature(OregonMessageBuffer, 55.5);
  setHumidity(OregonMessageBuffer, humidity);

  // Calculate the checksum
  calculateAndSetChecksum(OregonMessageBuffer);

  // Show the Oregon Message
  /*for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   {     Serial.print(OregonMessageBuffer[i] >> 4, HEX);
    Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
  }
    Serial.println();*/
  // Send the Message over RF
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(TWOTIME * 8);
  // Send a copie of the first message. The v2.1 protocol send the
  // message two time
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));

  SEND_LOW();

}
