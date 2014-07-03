#include <LiquidCrystal.h>
#include <TimedAction.h>
#include <NewPing.h>
#include <EmonLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

char ver[] = "v1.6"; 

//Ultrasonic sensor
#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 31 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
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
byte glyphs[5][8] = {  { B11111,B11111,B00000,B00000,B00000,B00000,B00000,B00000 },
                       { B00000,B00000,B00000,B00000,B00000,B00000,B11111,B11111 },  
                       { B11111,B11111,B00000,B00000,B00000,B00000,B11111,B11111 },  
                       { B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111 } ,  
                       { B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110 } };

// the width in characters of a big digit
// (excludes space between characters)
const int digitWidth = 3; 
//arrays  to index into custom characters that will comprise the big numbers 
const char bigDigitsTop[10][digitWidth]={ 3,0,3, 0,3,32, 2,2,3, 0,2,3, 3,1,3, 3,2,2, 3,2,2, 0,0,3,   3,2,3, 3,2,3};
const char bigDigitsBot[10][ digitWidth]={ 3,1,3, 1,3,1,  3,1,1, 1,1,3, 32,32,3, 1,1,3, 3,1,3, 32,32,3, 3,1,3, 1,1,3};
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

//action
TimedAction printLCDDataAction = TimedAction(400,printLCDData);
TimedAction ultrasonicSensorAction = TimedAction(5000,runUltrasonicSensor);
TimedAction energyMonitorAction = TimedAction(300, monitorEnergy); 
TimedAction brewTimerAction = TimedAction(1000, brewTimer);
TimedAction displayTimerAction = TimedAction(1000, displayTimerCountdown);
TimedAction boilerMonitorAction = TimedAction(1000, checkBoiler);
TimedAction brewMonitorAction = TimedAction(1000, checkPump);

void setup() {
    setupLCD();
    Serial.begin(9600);
    sensors.begin();    
    
    brewMonitor.current(brewMonitorPin, 111.11);
    boilerMonitor.current(boilerMonitorPin, 111.11);
    
    pinMode(screenPowerPin, OUTPUT);
    runUltrasonicSensor(); //lets get an inital waterlevel reading
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
   for(int i=0; i < 5; i++)    
   {
     lcd.createChar(i, glyphs[i]);
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

void loop() {

  energyMonitorAction.check();
  brewTimerAction.check();
  printLCDDataAction.check();
  displayTimerAction.check();
  ultrasonicSensorAction.check();
  boilerMonitorAction.check();
  brewMonitorAction.check();
  
  boilerTemperature = getTemp();
 
  //sendSensorData();
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
  if(pumpIrms > 0.60 && boilerTemperature > 100) 
  {  
       pumpIsOn = true;  
  }
  else
  {
       pumpIsOn = false;
  }
 
  //&& boilerTemperature > 50
  if(boilerIrms > 2)
  {
    if(boilerTemperature > 100)
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
  if(secondsBrewed > 0)
  {
    lcd.clear();
    showNumber(secondsBrewed, 2,1);
  }
  else
  {
    lcd.clear();
    showNumber(boilerTemperature, 0, 0);
  
    if(writeDroplet)
    { 
      lcd.createChar(6, droplet_glyph);
      lcd.setCursor(0,4);
      lcd.write(6);
    }    
    
    writeDroplet = true;
   
    lcd.setCursor(1,4);
    lcd.print(waterLevelPercent);
    lcd.write("%");
    lcd.setCursor(16,4);
    lcd.write(ver);    
  }
}
void displayTimerCountdown()
{
  if(screenTimerSeconds > 0)
  {
    screenTimerSeconds--;
  }
  
  if(screenTimerSeconds < 1)
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
  if(pumpIsOn)
  {
     secondsBrewed += 1;
  }
  else
  {
    secondsBrewed = 0;
  }
  
  if(secondsBrewed >= 99)
  {
    secondsBrewed = 1;
  }
}

void runUltrasonicSensor()
{
   unsigned int uS = sonar.ping();
   distance = uS / US_ROUNDTRIP_CM;

   if(distance <= watercontainerMinimumMeasurement)
   {
     waterLevelPercent = 100;
   }
   else
   {    
     if(distance >= watercontainerMaximumMeasurement)
     {
       waterLevelPercent = 0;
     }
     else
     {
       
       waterLevelPercent = map(watercontainerMaximumMeasurement - (distance - watercontainerMinimumMeasurement), watercontainerMinimumMeasurement,watercontainerMaximumMeasurement,0,100);
     }
   }
   
   if(waterLevelPercent < 25)
   {
     writeDroplet = false;
   }
   else
   {
     writeDroplet = true;
   }
}

float getTemp(){
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0); 
}
