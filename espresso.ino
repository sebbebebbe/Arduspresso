// includes
#include <LiquidCrystal.h>
#include <TimedAction.h>
#include <NewPing.h>
#include <EmonLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//onboard led
int ledPin = 13;

//Ultrasonic sensor
#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 31 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int ussInterval = 2000;
unsigned int distance;
unsigned int watercontainerHeight = 26;
unsigned int waterlevel;
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
int checkInterval = 100;
static byte droplet_glyph[] = {  B00100,  B00100,  B01110,  B01110,  B11111,  B11111,  B11111,  B01110 };
byte glyphs[5][8] = {  { B11111,B11111,B00000,B00000,B00000,B00000,B00000,B00000 },
                       { B00000,B00000,B00000,B00000,B00000,B00000,B11111,B11111 },  
                       { B11111,B11111,B00000,B00000,B00000,B00000,B11111,B11111 },  
                       { B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111 } ,  
                       { B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110 } };

// the width in characters of a big digit
// (excludes space between characters)
const int digitWidth = 3; 
//arrays  to index into custom characters that will comprise the big numbers 
// digits 0 - 4                             0      1       2      3      4 
const char bigDigitsTop[10][digitWidth]={ 3,0,3, 0,3,32, 2,2,3, 0,2,3, 3,1,3, 3,2,2, 3,2,2, 0,0,3,   3,2,3, 3,2,3};
const char bigDigitsBot[10][ digitWidth]={ 3,1,3, 1,3,1,  3,1,1, 1,1,3, 32,32,3, 1,1,3, 3,1,3, 32,32,3, 3,1,3, 1,1,3};
char buffer[12]; // used to convert a number into a string 
int screenPowerPin = 10;

//brewtimer
int secondsBrewed;
boolean pumpOn = false;
EnergyMonitor brewMonitor; // with 18ohms burden
int brewMonitorPin = 1;

//brewStatus
boolean boilerOn = false;
EnergyMonitor boilerMonitor; // with 18ohms burden
int boilerMonitorPin = 2;

//boilerTemp
int oneWirePin = 7;
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);
float boilerTemperature;

//genericTimer
int timerInterval = 1000;

//action
TimedAction printLCDDataAction = TimedAction(checkInterval,printLCDData);
TimedAction blinkAction = TimedAction(checkInterval, blinkLed);
TimedAction ultrasonicSensorAction = TimedAction(ussInterval,runUltrasonicSensor);
TimedAction timerAction = TimedAction(timerInterval, runTimer);
TimedAction screenOnAction = TimedAction(timerInterval, screenStatus); 

void setup() {
    setupLCD();
    Serial.begin(9600);
    pinMode(ledPin, HIGH);
    sensors.begin();    
    brewMonitor.current(brewMonitorPin, 111.11);
    boilerMonitor.current(boilerMonitorPin, 111.11);
    pinMode(screenPowerPin, OUTPUT);
  }

void setupLCD()
{
  // set up the LCD's number of columns and rows
   lcd.begin(20, 4);
   lcd.clear();
   delay(200);
   
   pinMode(contrastPin, OUTPUT); //contrast
   analogWrite(contrastPin, 30); // 0: maximum contrast  255: minimum
   createNumberGlyphs();   
   lcd.createChar(6, droplet_glyph);
   lcd.clear();
}

void createNumberGlyphs()
{
   for(int i=0; i < 5; i++)    
   {
     lcd.createChar(i, glyphs[i]);
   }
}

void loop() {
  
  //STUFF THAT NEEDS TO BE SET FAST
  boilerTemperature = getTemp();
  
  double IrmsBrewMonitor = brewMonitor.calcIrms(1480);
  //Serial.print("Pump:");
  //Serial.println(IrmsBrewMonitor*230.0);
  if((IrmsBrewMonitor*230.0) > 130)
  {
    pumpOn = true;
  }
  else
  {
    pumpOn = false;
  }
  
  double IrmsBoilerMonitor = boilerMonitor.calcIrms(1480);
  //Serial.print("Boiler:");
  //Serial.println(IrmsBoilerMonitor*230.0);
  if((IrmsBoilerMonitor*230.0) > 300)
  {
    boilerOn = true;
  }
  else
  {
    boilerOn = false;
  }
  
  
  blinkAction.check();
  ultrasonicSensorAction.check();
  printLCDDataAction.check();
  timerAction.check();
  screenOnAction.check();
}

void blinkLed()
{
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
}

void printLCDData()
{
  if(pumpOn)
  {
    lcd.clear();
    showNumber(secondsBrewed, 2,1);
  }
  else
  {
    lcd.clear();
    showNumber( floor(boilerTemperature), 0, 0);
  
    lcd.createChar(6, droplet_glyph);
    lcd.setCursor(0,4);
    lcd.write(6);
    lcd.setCursor(1,4);
    int waterLevelPercent = map(waterlevel,0,watercontainerHeight,0,100);
    lcd.print(waterLevelPercent);
    lcd.write("%");
  }
}

void screenStatus()
{
    if(boilerOn)
    {
      digitalWrite(screenPowerPin, HIGH);
    }
    else
    {
       digitalWrite(screenPowerPin, LOW);
    }
}

void runTimer()
{
  if(pumpOn)
  {
    secondsBrewed += 1;
  }
  else
  {
    secondsBrewed = 0;
  }
  
  if(secondsBrewed >= 99)
  {
    secondsBrewed = 0;
  }
}

void showDigit(int digit, int position, int shiftdown) 
{  
  lcd.setCursor(position * (digitWidth + 1), 0 + shiftdown);
    for(int i=0; i < digitWidth; i++)
      lcd.write(bigDigitsTop[digit][i]);  
    lcd.setCursor(position * (digitWidth + 1), 1 + shiftdown);  
    for(int i=0; i < digitWidth; i++)
      lcd.write(bigDigitsBot[digit][i]); 
}
   
void showNumber(int value, int position, int shiftdown) {  
  int index; // index to the digit being printed, 0 is the leftmost digit  
  itoa(value, buffer, 10); // see Recipe 2.8 for more on using itoa  
  // display each digit in sequence  
  for(index = 0; index < 10; index++) 
  {    char c = buffer[index];    
        if( c == 0)  // check for null (not the same as '0')      
          return; // the end of string character is a null, see Chapter 2    
          c = c - 48; // convert ascii value to a numeric value  (see Recipe 2.9)    
          showDigit(c, position + index, shiftdown);  
  }
} 

void runUltrasonicSensor()
{
   unsigned int uS = sonar.ping();
   distance = uS / US_ROUNDTRIP_CM;
   
   if(distance < 4)
   {
     //we consider the container full
     waterlevel = watercontainerHeight;
   }
   else
   {    
     if(distance > watercontainerHeight)
     {
       waterlevel = 0;
     }
     else
     {
       waterlevel = watercontainerHeight - distance;
     }
   }
}

float getTemp(){
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0); 
}
