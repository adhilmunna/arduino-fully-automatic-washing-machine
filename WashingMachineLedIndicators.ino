#include <EEPROM.h>
//#include<LiquidCrystal.h>
//LiquidCrystal lcd (7, 8, 9, 10, 11, 12);

//psensor and smoothing
//#include <Wire.h>
//#include <BMP180I2C.h>
//#define I2C_ADDRESS 0x77
#include <Smoothed.h>
Smoothed <float> pressureSensor;
//BMP180I2C bmp180(I2C_ADDRESS);

#define PRESSURE_SENSOR_PIN A4
#define BUTTON_PIN A2
char data = 0; 

const int waterLevelButton = 16; //A1
const int modeButton = 15; //A2
const int startStopButton = 2;

const int motorOnOffPin = 3;//6
const int motorDirectionPin = 5;
const int waterInletValvePin = 6;//5
const int drainValvePin = 4;

const int normalModeLed = 7;
const int heavyModeLed = 8;
const int dryModeLed = 9;
const int lowWaterLevelLed = 10;
const int mediumWaterLevelLed = 11;
const int fullWaterLevelLed = 12;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 250;    // the debounce time; increase if the output flickers
unsigned long calibDebounceDelay = 5000; 

int interruptedCount = 0; int play = 0; int pause = 0; int buttonValue=0;
int ssb = 0; int pssbState = 0; int cssbState = 0;
int mb = 0; int pmbState = 0; int cmbState = 0; int smode = 0; int mSelectorCount = 0;
int wlb = 0; int pwlbState = 0; int cwlbState = 0; int swl = 0; int wlSelectorCount = 0; int wlbState=0; //wlb ---> water level button
unsigned long buttonPressedDuration=0;

int toggled=0;
unsigned long buttonPressedAt=0;
unsigned long buttonReleasedAt=0;

int modeAccomplished = 0; unsigned long washModeStartTime = 0; unsigned long spinModeStartTime = 0; int washModeTimeLeft = 0; int spinModeTimeLeft = 0; 
int totalTimeLeft = 0;  int totalProcessTime = 0;
int WashCycleCount = 0; int completedWashCycleCount = 0; unsigned long WashCycleTime = 0;
int SpinCycleCount = 0; int completedSpinCycleCount = 0; unsigned long SpinCycleTime = 0;
int washModeStarted = 0; int spinModeStarted = 0; 

int isSoaking=0; int waitForSoakTime = 0; int currentSoakTime=0; int previousSoakTime=0; int soakTimeLeft = 0;

int serialPrintTimerPrevious=0; int serialPrintTimerCurrent = 0;

int WashNormalSpinState = 0; int WashRapidSpinState = 0; unsigned long MotorSpinStateChangeTime = 0; int drySpinState = 0;
int WashNormalSpinTime = 6; int WashNormalStopTime = 1; 
int WashRapidSpinTime = 750; int WashRapidStopTime = 375;

float pSensorMaximum = 0;
float pSensorMinimum = 0;
float pressureDifference = 170;
float smoothedSensorValueAvg = 0;
float actualWaterLevel = 0;
float requiredWaterLevel = 0;
float WLcurrentTime = 0;
float WLpreviousTime = 0;
int WLtimer = 0;
int ignoreWaterLevel = 0;
int washWaterLevelRequirementMet = 0; int spinWaterLevelRequirementMet = 0;
int spinRequest = 0;

float CalibWLtimer = 0;
float CalibWLcurrentTime = 0;
float CalibWLpreviousTime = 0;
float previousActualWaterLevel = 0;
float currentActualWaterLevel = 0;
int calibStatusThisCycle = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
float timer = 0;
unsigned long eepromTime = 0;
unsigned long eepromTimePrevious = 0;
int timerStarted = 0;
int timerPaused = 0;

//float totalRunTime = 0;
//float totalCompletedRunTime = 0;
//float percentageCompletion = 0;

int eepromValue1 = 0; //mode accomplished
int eepromValue2 = 0; //completed washcycle count
int eepromValue3 = 0; //completed spin cycle count
int eepromValue4 = 0; //interruptedCount play
int eepromValue5 = 1; //smode
int eepromValue6 = 2; //swl
int eepromValue7 = 0; //spinRequest
float eepromValue8 = 0; //timer
int eepromValue9 = 0; //washModeStartTime
int eepromValue10 = 0; //spinModeStartTime
int eepromValue11 = 0; //washModeStarted
int eepromValue12 = 0; //spinModeStarted
float eepromValue13 = 0; //pSensorMinimum
int eepromValue14 = 0; //WashNormalSpinState
int eepromValue15 = 0; //soakTimeLeft
float eepromValue16 = 0; //pressureDifference

//int ledCurrentMillis = 0;
//int ledpreviousMillis = 0;
//int ledTimer = 0;


void setup() {
//  EEPROM.write(15, 170);
  //drySpinState = 0;
  // LCD
  //lcd.begin(16, 2);
  //lcd.setCursor(0, 0);
  //lcd.print("Washing Machine");
  //lcd.setCursor(0, 1);
  //lcd.print("Starting....");

  pinMode(startStopButton, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(startStopButton), ssbPressed, FALLING);
  pinMode(waterLevelButton, INPUT);
  pinMode(modeButton, INPUT);

  pinMode(motorOnOffPin, OUTPUT); digitalWrite(motorOnOffPin, LOW);
  pinMode(motorDirectionPin, OUTPUT); digitalWrite(motorDirectionPin, LOW);
  pinMode(waterInletValvePin, OUTPUT); digitalWrite(waterInletValvePin, LOW);
  pinMode(drainValvePin, OUTPUT); digitalWrite(drainValvePin, LOW);

  pinMode(normalModeLed, OUTPUT);
  pinMode(heavyModeLed, OUTPUT);
  pinMode(dryModeLed, OUTPUT);
  pinMode(lowWaterLevelLed, OUTPUT);
  pinMode(mediumWaterLevelLed, OUTPUT);
  pinMode(fullWaterLevelLed, OUTPUT);

  Serial.begin(38400);
  
  eepromValue1 = EEPROM.read(0); modeAccomplished = eepromValue1; //Serial.print("modeAccomplished="); Serial.print(modeAccomplished); Serial.print("\n");
  eepromValue2 = EEPROM.read(1); completedWashCycleCount = eepromValue2; //Serial.print("completedWashCycleCount="); Serial.print(completedWashCycleCount); Serial.print("\n");
  eepromValue3 = EEPROM.read(2); completedSpinCycleCount = eepromValue3; //Serial.print("completedSpinCycleCount="); Serial.print(completedSpinCycleCount); Serial.print("\n");
  eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
  eepromValue5 = EEPROM.read(4); mSelectorCount = eepromValue5; //Serial.print("mSelectorCount="); Serial.print(mSelectorCount); Serial.print("\n"); if (mSelectorCount > 2){mSelectorCount = 0;}
  eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; //Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\n");  if (wlSelectorCount > 2){wlSelectorCount = 0;}
  eepromValue7 = EEPROM.read(6); spinRequest = eepromValue7; //Serial.print("spinRequest="); Serial.print(spinRequest); Serial.print("\n");
  eepromValue8 = EEPROM.read(7); timer = eepromValue8 * 60; //Serial.print("timer="); Serial.print(timer); Serial.print("\n");
  eepromValue9 = EEPROM.read(8); washModeStartTime = eepromValue9 * 60; //Serial.print("washModeStartTime="); Serial.print(washModeStartTime); Serial.print("\n");
  eepromValue10 = EEPROM.read(9); spinModeStartTime = eepromValue10 * 60; //Serial.print("spinModeStartTime="); Serial.print(spinModeStartTime); Serial.print("\n");
  eepromValue11 = EEPROM.read(10); washModeStarted = eepromValue11; //Serial.print("washModeStarted="); Serial.print(washModeStarted); Serial.print("\n");
  eepromValue12 = EEPROM.read(11); spinModeStarted = eepromValue12; //Serial.print("spinModeStarted="); Serial.print(spinModeStarted); Serial.print("\n");
  eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
  eepromValue14 = EEPROM.read(13); WashNormalSpinState = eepromValue14; //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\n");
  eepromValue15 = EEPROM.read(14); soakTimeLeft = eepromValue15*60; //Serial.print("soakTimeLeft="); Serial.print(soakTimeLeft); Serial.print("\n");
  eepromValue16 = EEPROM.read(15); pressureDifference = eepromValue16; //Serial.print("pressureDifference="); Serial.print(pressureDifference); Serial.print("\n");  
  

    {
    Serial.print("Paused"); Serial.print("|"); Serial.print(""); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected mode= "); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Wash count left= "); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected soak time= "); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Soak time left= "); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected water level="); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Current water level="); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("CalibWLtimer = "); Serial.print("|"); Serial.print(""); Serial.print("|");
    } 

  //BMP180 AND SMOOTHING
  { pressureSensor.begin(SMOOTHED_AVERAGE, 50);
    pressureSensor.clear();
//    while (!Serial);
//    Wire.begin();
//    if (!bmp180.begin())
//    {
//      Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
//      while (1);
//    }
//    bmp180.resetToDefaults();
//    bmp180.setSamplingMode(BMP180MI::MODE_UHR);
//    Serial.println("Pressure sensor initiated!");
  }

  digitalWrite(normalModeLed, HIGH);  delay(250); digitalWrite(heavyModeLed, HIGH);  delay(250); digitalWrite(dryModeLed, HIGH);  delay(250);
  digitalWrite(lowWaterLevelLed, HIGH);  delay(250); digitalWrite(mediumWaterLevelLed, HIGH);  delay(250); digitalWrite(fullWaterLevelLed, HIGH);  delay(250);
  digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, LOW);
  digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, LOW);
}

void loop() {

  btSerialRead();
  pressureSensorRead();

  serialPrintTimerCurrent = millis();
  if (serialPrintTimerCurrent-serialPrintTimerPrevious>=1000) {serialPrints();   Serial.print ("\n"); serialPrintTimerPrevious=serialPrintTimerCurrent;}

  ssbPressed();
  executeSelection();
  timerFunction();
  eepromTimer();

  

}

void executeSelection() {
    if (buttonValue<700)
    {
//      Serial.print("Nothing pressed "); Serial.println(buttonValue);
      mb = 0;
      ssb = 0;
      wlb = 0;
    }

  if (play == 1)
  {
    if (mSelectorCount == 0) {
      NormalMode(); //Rinse
    }
    if (mSelectorCount == 1) {
      HeavyMode(); //Wash
    }
    if (mSelectorCount == 2) {
      DryMode(); //Spin
    }
  }

  if (play == 0)
  {
    drySpinState = 0;
    waterValveOff();
    drainValveOff();
    stopMotor();
    wlSelector();
    mSelector();
    //lcd.setCursor(0, 1);
    //lcd.print("Paused          ");
//    Serial.print("Paused"); Serial.print("\t");
  }

    {//led commands,
    if (mSelectorCount == 0)
    {
      digitalWrite(normalModeLed, HIGH); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, LOW);
    }

    if (mSelectorCount == 1)
    {
      digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, HIGH); digitalWrite(dryModeLed, LOW);
    }
    if (mSelectorCount == 2)
    {
      digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, HIGH);
    }


    if (wlSelectorCount == 0)
    {
      swl = 25; digitalWrite(lowWaterLevelLed, HIGH); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, LOW);
    }

    if (wlSelectorCount == 1)
    {
      swl = 50; digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, HIGH); digitalWrite(fullWaterLevelLed, LOW);
    }
    if (wlSelectorCount == 2)
    {
      swl = 95; digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, HIGH);
    }
    if (wlSelectorCount == 3)
    {
      swl = 75; digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, HIGH);
    }
    
    }


}

//BLUETOOTH AND SERIAL
void btSerialRead(){
if(Serial.available() > 0)  
{
data = Serial.read();     
               
if(data == 'a') {  interruptedCount = 0; interruptedCount = interruptedCount + 1;}             
else if(data == 'b') { interruptedCount = 1; interruptedCount = interruptedCount + 1;}  
else if(data == 'r') { mSelectorCount = 0; resetValues();}  
else if(data == 'w') { mSelectorCount = 1; resetValues();}     
else if(data == 's') { mSelectorCount = 2; resetValues();}  
//else if(data == 'f') { mSelectorCount = 2;}  
else if(data == 'g') { waitForSoakTime=0;soakTimeLeft = waitForSoakTime;}   
else if(data == 'h') { waitForSoakTime=120;soakTimeLeft = waitForSoakTime;}  
else if(data == 'i') { waitForSoakTime=300;soakTimeLeft = waitForSoakTime;}  
else if(data == 'j') { waitForSoakTime=600;soakTimeLeft = waitForSoakTime;}   
else if(data == 'k') { waitForSoakTime=900;soakTimeLeft = waitForSoakTime;}  

else if(data == 'l') { wlSelectorCount = 0;}  
else if(data == 'm') { wlSelectorCount = 1;} 
else if(data == 'n') { wlSelectorCount = 3;}  
else if(data == 'o') { wlSelectorCount = 2;}  

else if(data == 'p') { pressureSensorCalibrationBottom();}  
else if(data == 'q') { pressureSensorCalibrationTop();}    
}                            
}

void serialPrints(){


  if (play == 0){
    Serial.print("Paused"); Serial.print("|"); Serial.print(""); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected mode= "); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Wash count left= "); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected soak time= "); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Soak time left= "); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("Selected water level="); Serial.print("|");Serial.print(""); Serial.print("|"); Serial.print("Current water level="); Serial.print("|");Serial.print(""); Serial.print("|");
    Serial.print("CalibWLtimer = "); Serial.print("|"); Serial.print(""); Serial.print("|");
    } 
    
  if (play == 1)
    {
      //Status//
      Serial.print("Playing"); Serial.print("|");
      
      //Process//
      if (spinRequest == 0 && ignoreWaterLevel == 0) {Serial.print("Filling Water");Serial.print("|"); }
      if (spinRequest == 0 && ignoreWaterLevel == 1) {if (isSoaking == 1) {Serial.print("Soaking");Serial.print("|"); }if (isSoaking == 0) {Serial.print("Washing"); Serial.print("|");}}
      if (spinRequest == 1 && ignoreWaterLevel == 0) {Serial.print("Draining");Serial.print("|"); }
      if (spinRequest == 1 && ignoreWaterLevel == 1) {Serial.print("Spinning"); Serial.print("|");}

      //Time left
      Serial.print(washModeTimeLeft+spinModeTimeLeft+soakTimeLeft); Serial.print("|"); 
      
      //Mode//
      if (mSelectorCount == 0) {Serial.print("Selected mode= "); Serial.print("|"); Serial.print("Rinse"); Serial.print("|"); }
      if (mSelectorCount == 1) {Serial.print("Selected mode= "); Serial.print("|"); Serial.print("Wash"); Serial.print("|");}
      if (mSelectorCount == 2) {Serial.print("Selected mode= "); Serial.print("|"); Serial.print("Spin"); Serial.print("|");}
      
      //Wash count left//
      Serial.print("Cycles left= "); Serial.print("|"); Serial.print(WashCycleCount - completedWashCycleCount); Serial.print("|");
      
      //selected soak time//
      Serial.print("Selected soak time= "); Serial.print("|"); Serial.print(waitForSoakTime); Serial.print("|");
      
      //soak time left//
      Serial.print("Soak time left= "); Serial.print("|"); Serial.print(soakTimeLeft); Serial.print("|");
      
      //Selected water level//
      Serial.print("Selected water level="); Serial.print("|"); Serial.print(requiredWaterLevel); Serial.print("|");
      
      //Current water level//
      Serial.print("Current water level="); Serial.print("|"); Serial.print(actualWaterLevel); Serial.print("|");

      //Calib Timer//
      Serial.print("CalibWLtimer = "); Serial.print("|"); Serial.print(CalibWLtimer); Serial.print("|");
            
//Timer
        //     Serial.print("Timer = " ); Serial.print(timer); Serial.print("\t" );
        
        //pressureSensorRead
        //    Serial.print("Psr="); //Serial.print(currentSensorValue); //Serial.print("\t=");
        //    Serial.print("Pss="); Serial.print(smoothedSensorValueAvg); Serial.print("\t=");
        //    Serial.print("Pssm="); Serial.print(actualWaterLevel); Serial.print("\t");
        
        //WaterLevelMonitor
//              Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\t");
//              Serial.print("swl="); Serial.print(swl); Serial.print("\t"); 
//              Serial.print("CalibWLtimer="); Serial.print(CalibWLtimer); Serial.print("\t");
        //    Serial.print("currentActualWaterLevel="); Serial.print(currentActualWaterLevel); Serial.print("\t");
        //    Serial.print("previousActualWaterLevel="); Serial.print(previousActualWaterLevel); Serial.print("\t");
        
        //Running process
  //            Serial.print("WashModeTimeLeft="); Serial.print(washModeTimeLeft); Serial.print("\t");
//              Serial.print("spinModeTimeLeft="); Serial.print(spinModeTimeLeft); Serial.print("\t");

    }

//Serial.print("smoothedSensorValueAvg="); Serial.print(smoothedSensorValueAvg); Serial.print("\t");           
//Serial.print("actualWaterLevel="); Serial.print(actualWaterLevel); Serial.print("\t");
//Serial.print("pSensorMinimun="); Serial.print(pSensorMinimum); Serial.print("\t");
//Serial.print("pSensorMaximum="); Serial.print(pSensorMaximum); Serial.print("\t");
//Serial.print("pressureDifference="); Serial.print(pressureDifference); Serial.print("\t");

}



//EEPROM Activities
void eepromTimer() {
  if (timer - eepromTime > 60)
  {
    eepromTime = timer;
    eepromWrite();
  }
}

void eepromWrite() {

  if (modeAccomplished == 0)
  {
    serialPrints(); Serial.print("writing to eeprom\n"); //Serial.print(eepromTime); //Serial.print("\t");
    EEPROM.write(0, modeAccomplished);
    EEPROM.write(1, completedWashCycleCount);
    EEPROM.write(2, completedSpinCycleCount);
    EEPROM.write(3, interruptedCount);
    EEPROM.write(4, mSelectorCount);
    EEPROM.write(5, wlSelectorCount);
    EEPROM.write(6, spinRequest);
    EEPROM.write(7, (timer / 60));
    EEPROM.write(8, washModeStartTime / 60);
    EEPROM.write(9, spinModeStartTime / 60);
    EEPROM.write(10, washModeStarted);
    EEPROM.write(11, spinModeStarted);
    EEPROM.write(12, pSensorMinimum / 4);
    EEPROM.write(13, WashNormalSpinState);
    EEPROM.write(14, soakTimeLeft/60);

    //Serial.print("timer to eeprom="); //Serial.print(timer / 10); //Serial.print("\n");

  eepromValue1 = EEPROM.read(0); modeAccomplished = eepromValue1; //Serial.print("modeAccomplished="); Serial.print(modeAccomplished); Serial.print("\n");
  eepromValue2 = EEPROM.read(1); completedWashCycleCount = eepromValue2; //Serial.print("completedWashCycleCount="); Serial.print(completedWashCycleCount); Serial.print("\n");
  eepromValue3 = EEPROM.read(2); completedSpinCycleCount = eepromValue3; //Serial.print("completedSpinCycleCount="); Serial.print(completedSpinCycleCount); Serial.print("\n");
  eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
  eepromValue5 = EEPROM.read(4); mSelectorCount = eepromValue5; //Serial.print("mSelectorCount="); Serial.print(mSelectorCount); Serial.print("\n");
  eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; //Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\n");
  eepromValue7 = EEPROM.read(6); spinRequest = eepromValue7; //Serial.print("spinRequest="); Serial.print(spinRequest); Serial.print("\n");
  eepromValue8 = EEPROM.read(7); timer = eepromValue8 * 60; //Serial.print("timer="); Serial.print(timer); Serial.print("\n");
  eepromValue9 = EEPROM.read(8); washModeStartTime = eepromValue9 * 60; //Serial.print("washModeStartTime="); Serial.print(washModeStartTime); Serial.print("\n");
  eepromValue10 = EEPROM.read(9); spinModeStartTime = eepromValue10 * 60; //Serial.print("spinModeStartTime="); Serial.print(spinModeStartTime); Serial.print("\n");
  eepromValue11 = EEPROM.read(10); washModeStarted = eepromValue11; //Serial.print("washModeStarted="); Serial.print(washModeStarted); Serial.print("\n");
  eepromValue12 = EEPROM.read(11); spinModeStarted = eepromValue12; //Serial.print("spinModeStarted="); Serial.print(spinModeStarted); Serial.print("\n");
  eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
  eepromValue14 = EEPROM.read(13); WashNormalSpinState = eepromValue14; //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\n");
  eepromValue15 = EEPROM.read(14); soakTimeLeft = eepromValue15*60; //Serial.print("soakTimeLeft="); Serial.print(soakTimeLeft); Serial.print("\n");
  eepromValue16 = EEPROM.read(15); pressureDifference = eepromValue16; //Serial.print("pressureDifference="); Serial.print(pressureDifference); Serial.print("\n");  
  
  }

  if (modeAccomplished == 1)
  {
    EEPROM.write(0, 0);
    EEPROM.write(1, 0);
    EEPROM.write(2, 0);
    EEPROM.write(3, 0);
    EEPROM.write(4, 0);
    EEPROM.write(5, 0);
    EEPROM.write(6, 0);
    EEPROM.write(7, 0);
    EEPROM.write(8, 0);
    EEPROM.write(9, 0);
    EEPROM.write(10, 0);
    EEPROM.write(11, 0);
    EEPROM.write(13, 0);
    EEPROM.write(14, waitForSoakTime/60);
    //Serial.print("mode accomplished"); //Serial.print ("\t");

    digitalWrite(normalModeLed, HIGH);  digitalWrite(heavyModeLed, HIGH);  digitalWrite(dryModeLed, HIGH);
    digitalWrite(lowWaterLevelLed, HIGH);  digitalWrite(mediumWaterLevelLed, HIGH);  digitalWrite(fullWaterLevelLed, HIGH);
    delay(1000);

    digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, LOW);
    digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, LOW);
    delay(1000);

    digitalWrite(normalModeLed, HIGH);  digitalWrite(heavyModeLed, HIGH);  digitalWrite(dryModeLed, HIGH);
    digitalWrite(lowWaterLevelLed, HIGH);  digitalWrite(mediumWaterLevelLed, HIGH);  digitalWrite(fullWaterLevelLed, HIGH);
    delay(1000);

    digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, LOW);
    digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, LOW);
    delay(1000);

    digitalWrite(normalModeLed, HIGH);  digitalWrite(heavyModeLed, HIGH);  digitalWrite(dryModeLed, HIGH);
    digitalWrite(lowWaterLevelLed, HIGH);  digitalWrite(mediumWaterLevelLed, HIGH);  digitalWrite(fullWaterLevelLed, HIGH);
    delay(1000);

    digitalWrite(normalModeLed, LOW); digitalWrite(heavyModeLed, LOW); digitalWrite(dryModeLed, LOW);
    digitalWrite(lowWaterLevelLed, LOW); digitalWrite(mediumWaterLevelLed, LOW); digitalWrite(fullWaterLevelLed, LOW);
    delay(1000);

    pressureSensorCalibrationBottom();
    modeAccomplished = 0;
  }

}

void resetValues(){
      timer = 0;
      completedWashCycleCount = 0;
      completedSpinCycleCount = 0;
      spinRequest = 0;
      washModeStartTime = 0;
      spinModeStartTime = 0;
      washModeTimeLeft = 0;
      spinModeTimeLeft = 0;
      washModeStarted = 0;
      spinModeStarted = 0;
      ignoreWaterLevel = 0;
      WashNormalSpinState = 0;
      waitForSoakTime=0;
      soakTimeLeft = waitForSoakTime;
      modeAccomplished = 0;

      EEPROM.write(0, 0);
      EEPROM.write(1, 0);
      EEPROM.write(2, 0);
      EEPROM.write(3, 0);
      EEPROM.write(4, mSelectorCount);
      EEPROM.write(5, wlSelectorCount);
      EEPROM.write(6, 0);
      EEPROM.write(7, 0);
      EEPROM.write(8, 0);
      EEPROM.write(9, 0);
      EEPROM.write(10, 0);
      EEPROM.write(11, 0);
      EEPROM.write(12, 0);
      EEPROM.write(13, WashNormalSpinState);
      EEPROM.write(14, soakTimeLeft/60);
      eepromValue1 = EEPROM.read(0); modeAccomplished = eepromValue1; //Serial.print("modeAccomplished="); Serial.print(modeAccomplished); Serial.print("\n");
      eepromValue2 = EEPROM.read(1); completedWashCycleCount = eepromValue2; //Serial.print("completedWashCycleCount="); Serial.print(completedWashCycleCount); Serial.print("\n");
      eepromValue3 = EEPROM.read(2); completedSpinCycleCount = eepromValue3; //Serial.print("completedSpinCycleCount="); Serial.print(completedSpinCycleCount); Serial.print("\n");
      eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
      eepromValue5 = EEPROM.read(4); mSelectorCount = eepromValue5; //Serial.print("mSelectorCount="); Serial.print(mSelectorCount); Serial.print("\n");
      eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; //Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\n");
      eepromValue7 = EEPROM.read(6); spinRequest = eepromValue7; //Serial.print("spinRequest="); Serial.print(spinRequest); Serial.print("\n");
      eepromValue8 = EEPROM.read(7); timer = eepromValue8 * 60; //Serial.print("timer="); Serial.print(timer); Serial.print("\n");
      eepromValue9 = EEPROM.read(8); washModeStartTime = eepromValue9 * 60; //Serial.print("washModeStartTime="); Serial.print(washModeStartTime); Serial.print("\n");
      eepromValue10 = EEPROM.read(9); spinModeStartTime = eepromValue10 * 60; //Serial.print("spinModeStartTime="); Serial.print(spinModeStartTime); Serial.print("\n");
      eepromValue11 = EEPROM.read(10); washModeStarted = eepromValue11; //Serial.print("washModeStarted="); Serial.print(washModeStarted); Serial.print("\n");
      eepromValue12 = EEPROM.read(11); spinModeStarted = eepromValue12; //Serial.print("spinModeStarted="); Serial.print(spinModeStarted); Serial.print("\n");
      eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
      eepromValue14 = EEPROM.read(13); WashNormalSpinState = WashNormalSpinState; //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\n");
      eepromValue15 = EEPROM.read(14); soakTimeLeft = eepromValue15*60; //Serial.print("soakTimeLeft="); Serial.print(soakTimeLeft); Serial.print("\n");
}

//Timer
void timerFunction() {
 // Serial.print("Timer = " ); Serial.print(timer); Serial.print("\t" );
  currentMillis = millis();

  if (play == 1)
  {
    if (spinRequest == 0 && washWaterLevelRequirementMet == 0)
    {
      //  //Serial.print("Paused" );
      //    //Serial.print("\t" );
      timerPaused = 1;
      timerStarted = 0;
    }

    if (spinRequest == 0 && washWaterLevelRequirementMet == 1)
    {
      //  //Serial.print("playing" );
      //    //Serial.print("\t" );
      timerPaused = 0;
      timerStarted = 1;
    }
    if (spinRequest == 1 && spinWaterLevelRequirementMet == 0)
    {
      //  //Serial.print("Paused" );
      //    //Serial.print("\t" );
      timerPaused = 1;
      timerStarted = 0;
    }
    if (spinRequest == 1 && spinWaterLevelRequirementMet == 1)
    {
      //  //Serial.print("Paused" );
      //    //Serial.print("\t" );
      timerPaused = 0;
      timerStarted = 1;
    }
  }
  if (play == 0)
  {
    //  //Serial.print("Paused" );
    //    //Serial.print("\t" );
    timerPaused = 1;
    timerStarted = 0;
  }
  
  if (timerStarted == 1)
  {
    if ((currentMillis - previousMillis) >= 1000)
    {
      timer = timer + 1;
      previousMillis = currentMillis;
    }
  }
}

void totalTimerFunction() {
 totalProcessTime =  WashCycleCount * (WashCycleTime + SpinCycleTime);
 totalTimeLeft = 0;
}


//Selection and display
void wlSelector() {
     
 //buttonValue=analogRead(BUTTON_PIN); 
   if (buttonValue>1020)
      {
      Serial.print("W pressed "); Serial.println(buttonValue);
        mb = 0;
        ssb = 0;
        wlb = 1;
      }
   else
   {
      wlb = 0; 
   }
   
  if (wlb == 1)
    {
      buttonPressedAt= millis();
      toggled=0;
      //Serial.print ("buttonPressedDuration ="); Serial.print (buttonPressedDuration) ; Serial.print ("\t"); Serial.print ("wlbState ="); Serial.println (wlbState) ; 

        if ( (buttonPressedDuration > debounceDelay) && (buttonPressedDuration < 5000) && (toggled==0))
        {

            //Serial.print("pressed"); Serial.print("\n");
           // Serial.print(wlSelectorCount); Serial.print("\n");
            wlSelectorCount = wlSelectorCount + 1;
              if (wlSelectorCount > 2)
                {
                  wlSelectorCount = 0;
                }
            ignoreWaterLevel = 0;
            EEPROM.write(5, wlSelectorCount);  
            toggled=1;
            
            eepromValue1 = EEPROM.read(0); modeAccomplished = eepromValue1; //Serial.print("modeAccomplished="); Serial.print(modeAccomplished); Serial.print("\n");
            eepromValue2 = EEPROM.read(1); completedWashCycleCount = eepromValue2; //Serial.print("completedWashCycleCount="); Serial.print(completedWashCycleCount); Serial.print("\n");
            eepromValue3 = EEPROM.read(2); completedSpinCycleCount = eepromValue3; //Serial.print("completedSpinCycleCount="); Serial.print(completedSpinCycleCount); Serial.print("\n");
            eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
            eepromValue5 = EEPROM.read(4); mSelectorCount = eepromValue5; //Serial.print("mSelectorCount="); Serial.print(mSelectorCount); Serial.print("\n");
            eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; //Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\n");
            eepromValue7 = EEPROM.read(6); spinRequest = eepromValue7; //Serial.print("spinRequest="); Serial.print(spinRequest); Serial.print("\n");
            eepromValue8 = EEPROM.read(7); timer = eepromValue8 * 60; //Serial.print("timer="); Serial.print(timer); Serial.print("\n");
            eepromValue9 = EEPROM.read(8); washModeStartTime = eepromValue9 * 60; //Serial.print("washModeStartTime="); Serial.print(washModeStartTime); Serial.print("\n");
            eepromValue10 = EEPROM.read(9); spinModeStartTime = eepromValue10 * 60; //Serial.print("spinModeStartTime="); Serial.print(spinModeStartTime); Serial.print("\n");
            eepromValue11 = EEPROM.read(10); washModeStarted = eepromValue11; //Serial.print("washModeStarted="); Serial.print(washModeStarted); Serial.print("\n");
            eepromValue12 = EEPROM.read(11); spinModeStarted = eepromValue12; //Serial.print("spinModeStarted="); Serial.print(spinModeStarted); Serial.print("\n");
            eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
            eepromValue14 = EEPROM.read(13); WashNormalSpinState = WashNormalSpinState; //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\n");
            
            buttonPressedDuration = 0;
        }

        if (buttonPressedDuration > 5000 && buttonPressedDuration < 10000 )
        {
          serialPrints(); Serial.println ("Calibrated");
          pressureSensorCalibrationBottom();
          buttonPressedDuration = 0;
        }  

        if (buttonPressedDuration > 10000)
        {
          serialPrints(); Serial.println ("Calibrated");
          pressureSensorCalibrationTop();
          buttonPressedDuration = 0;
        }  
    }

  if (wlb == 0 )
    {
      buttonReleasedAt= millis();
      buttonPressedDuration = buttonReleasedAt-buttonPressedAt;
    }

}

void mSelector() {
     
 //buttonValue=analogRead(BUTTON_PIN); 
  if (buttonValue>1000 && buttonValue<1020)
      {
//       Serial.print("M pressed "); Serial.println(buttonValue);
        mb = 1;
        ssb = 0;
        wlb = 0;
      }
   else
   {
      mb = 0; 
   }
   
  if (mb == 1)
    {
      buttonPressedAt= millis();
      toggled=0;
      //Serial.print ("buttonPressedDuration ="); Serial.print (buttonPressedDuration) ; Serial.print ("\t"); Serial.print ("wlbState ="); Serial.println (wlbState) ; 

        if ( (buttonPressedDuration > debounceDelay) && (buttonPressedDuration < 5000) && (toggled==0))
        {

            //Serial.print("pressed"); Serial.print("\n");
           // Serial.print(wlSelectorCount); Serial.print("\n");
            mSelectorCount = mSelectorCount + 1;
              if (mSelectorCount > 2)
                {
                  mSelectorCount = 0;
                }
            resetValues();
            toggled=1;
            
            eepromValue1 = EEPROM.read(0); modeAccomplished = eepromValue1; //Serial.print("modeAccomplished="); Serial.print(modeAccomplished); Serial.print("\n");
            eepromValue2 = EEPROM.read(1); completedWashCycleCount = eepromValue2; //Serial.print("completedWashCycleCount="); Serial.print(completedWashCycleCount); Serial.print("\n");
            eepromValue3 = EEPROM.read(2); completedSpinCycleCount = eepromValue3; //Serial.print("completedSpinCycleCount="); Serial.print(completedSpinCycleCount); Serial.print("\n");
            eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
            eepromValue5 = EEPROM.read(4); mSelectorCount = eepromValue5; //Serial.print("mSelectorCount="); Serial.print(mSelectorCount); Serial.print("\n");
            eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; //Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\n");
            eepromValue7 = EEPROM.read(6); spinRequest = eepromValue7; //Serial.print("spinRequest="); Serial.print(spinRequest); Serial.print("\n");
            eepromValue8 = EEPROM.read(7); timer = eepromValue8 * 60; //Serial.print("timer="); Serial.print(timer); Serial.print("\n");
            eepromValue9 = EEPROM.read(8); washModeStartTime = eepromValue9 * 60; //Serial.print("washModeStartTime="); Serial.print(washModeStartTime); Serial.print("\n");
            eepromValue10 = EEPROM.read(9); spinModeStartTime = eepromValue10 * 60; //Serial.print("spinModeStartTime="); Serial.print(spinModeStartTime); Serial.print("\n");
            eepromValue11 = EEPROM.read(10); washModeStarted = eepromValue11; //Serial.print("washModeStarted="); Serial.print(washModeStarted); Serial.print("\n");
            eepromValue12 = EEPROM.read(11); spinModeStarted = eepromValue12; //Serial.print("spinModeStarted="); Serial.print(spinModeStarted); Serial.print("\n");
            eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
            eepromValue14 = EEPROM.read(13); WashNormalSpinState = WashNormalSpinState; //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\n");
            
            buttonPressedDuration = 0;
        }

    }

  if (mb == 0 )
    {
      buttonReleasedAt= millis();
      buttonPressedDuration = buttonReleasedAt-buttonPressedAt;
    }

}

void ssbPressed() {
 // buttonValue=analogRead(BUTTON_PIN);
      if (buttonValue>800 && buttonValue<900)
      {
//        Serial.print("P pressed "); Serial.println(buttonValue);
        mb = 0;
        ssb = 1;
        wlb = 0;
      }
     else
      {
        ssb = 0;
      }
  
  if (ssb == 1)
  {
    //Serial.print("not Pressed");
    lastDebounceTime = 0;
  }
      
  if (ssb != pssbState )
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (ssb != cssbState)
    {
      cssbState = ssb;
      if (cssbState == 1)
      {
        //Serial.print("pressed"); Serial.print("\n");
        interruptedCount = interruptedCount + 1;

          if (interruptedCount == 0)
          {
            play = 0;
            EEPROM.write(3, interruptedCount);
            eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");

          }
        
          if (interruptedCount == 1)
          {
            play = 1;
            EEPROM.write(3, interruptedCount);
            eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
          }
          
          if (interruptedCount > 1)
          {
            interruptedCount = 0;
            play = 0;
            EEPROM.write(3, interruptedCount);
            eepromValue4 = EEPROM.read(3); interruptedCount = eepromValue4; //Serial.print("play interruptedCount="); Serial.print(interruptedCount); Serial.print("\n");
          }
      }
    }
  }
  pssbState = ssb;

  if (interruptedCount == 0)
  {
    play = 0;
  }

  if (interruptedCount == 1)
  {
    play = 1;
  }
  
  if (interruptedCount > 1)
  {
    interruptedCount = 0;
    play = 0;
  }
}


//Water level activity
void pressureSensorRead() {
//  if (!bmp180.measurePressure())
//  {
//    serialPrints(); Serial.println("could not start perssure measurement, is a measurement already running?");
//    return;
//  }
//  do
//  {
//    //    delay(1000);
//  } while (!bmp180.hasValue());
//  float currentSensorValue = bmp180.getPressure();

  float currentSensorValue = analogRead(PRESSURE_SENSOR_PIN);
  pressureSensor.add(currentSensorValue);
  smoothedSensorValueAvg = pressureSensor.get();
  //    //Serial.print(currentSensorValue);
  float lastValueStoredAvg = pressureSensor.getLast();
  pSensorMaximum = (pSensorMinimum - pressureDifference) ;
  actualWaterLevel = map(smoothedSensorValueAvg, pSensorMinimum , pSensorMaximum, 0, 100);
  //Serial.print("Psr="); //Serial.print(currentSensorValue); //Serial.print("\t=");
  //Serial.print("Pss="); Serial.print(smoothedSensorValueAvg); Serial.print("\t=");
  //    Serial.print("Pssm="); Serial.print(actualWaterLevel); Serial.print("\t");
}

void pressureSensorCalibrationBottom() {
  serialPrints(); Serial.println("Running pressure sensor calibration");
//  if (!bmp180.measurePressure())
//  {
//    serialPrints(); Serial.println("could not start perssure measurement, is a measurement already running?");
//    return;
//  }
//  do
//  {
//    //    delay(1000);
//  } while (!bmp180.hasValue());
//  float currentSensorValue = bmp180.getPressure();

  float currentSensorValue = analogRead(PRESSURE_SENSOR_PIN);
  pressureSensor.add(currentSensorValue);
  smoothedSensorValueAvg = pressureSensor.get();
  //Serial.print(currentSensorValue);
  float lastValueStoredAvg = pressureSensor.getLast();
  pSensorMinimum = smoothedSensorValueAvg;
  pSensorMaximum = (pSensorMinimum - pressureDifference) ;
  EEPROM.write(12, pSensorMinimum / 4);
  eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n");
  serialPrints(); Serial.print("New pSensorMinimum = "); Serial.print(pSensorMinimum); Serial.println("");
  

  CalibWLtimer = 0;
  CalibWLpreviousTime =0; 
  CalibWLcurrentTime=0;
  previousActualWaterLevel=0;
  currentActualWaterLevel=0;
  ignoreWaterLevel = 0;
}

void pressureSensorCalibrationTop() {
  serialPrints(); Serial.println("Running pressure sensor calibration");
//  if (!bmp180.measurePressure())
//  {
//    serialPrints(); Serial.println("could not start perssure measurement, is a measurement already running?");
//    return;
//  }
//  do
//  {
//    //    delay(1000);
//  } while (!bmp180.hasValue());
//  float currentSensorValue = bmp180.getPressure();

  float currentSensorValue = analogRead(PRESSURE_SENSOR_PIN);
  pressureSensor.add(currentSensorValue);
  smoothedSensorValueAvg = pressureSensor.get();
  //Serial.print(currentSensorValue);
  float lastValueStoredAvg = pressureSensor.getLast();
  pSensorMaximum = smoothedSensorValueAvg;
  eepromValue13 = EEPROM.read(12); pSensorMinimum = eepromValue13 * 4; //Serial.print("pSensorMinimum="); Serial.print(pSensorMinimum); Serial.print("\n"); 
  serialPrints(); Serial.print("New pSensorMaximum = "); Serial.print(pSensorMaximum); Serial.println("");
  EEPROM.write(15, pressureDifference);
  eepromValue16 = EEPROM.read(15); pressureDifference = eepromValue16; //Serial.print("pressureDifference="); Serial.print(pressureDifference); Serial.print("\n");  
  pSensorMinimum = pSensorMaximum + pressureDifference;
  
  CalibWLtimer = 0;
  CalibWLpreviousTime =0; 
  CalibWLcurrentTime=0;
  previousActualWaterLevel=0;
  currentActualWaterLevel=0;
  ignoreWaterLevel = 0;
}


void washWaterLevelMonitor() {
  WLcurrentTime = millis();
  CalibWLcurrentTime = millis();
  requiredWaterLevel = swl;
  pressureSensorRead();
  currentActualWaterLevel = actualWaterLevel;
  eepromValue6 = EEPROM.read(5); wlSelectorCount = eepromValue6; 
//  Serial.print("wlSelectorCount="); Serial.print(wlSelectorCount); Serial.print("\t");
//  Serial.print("swl="); Serial.print(swl); Serial.print("\t");
//  Serial.print("requiredWaterLevel="); Serial.print(requiredWaterLevel); Serial.print("\t");
//  Serial.print("actualWaterLevel="); Serial.print(actualWaterLevel); Serial.print("\t");
//
////  Serial.print("currentActualWaterLevel="); Serial.print(currentActualWaterLevel); Serial.print("\t");
////  Serial.print("previousActualWaterLevel="); Serial.print(previousActualWaterLevel); Serial.print("\t");

  if (ignoreWaterLevel == 0)
  {
    if (actualWaterLevel < requiredWaterLevel) // if water in drum is less than selected level
    {
      WLtimer = 0;
      drainValveOff();
      waterValveOn(); //Turn on water valve
      washWaterLevelRequirementMet = 0; //water level requirement is not met
//      Serial.print("Filling water\t");
      //lcd.setCursor(0, 1);
      //lcd.print("Filling      ");
    }

    if (actualWaterLevel > requiredWaterLevel) // if water in drum is more than selected level
    {
      if ((WLcurrentTime - WLpreviousTime) >= 1000)
      {
        WLtimer = WLtimer + 1;
        WLpreviousTime = WLcurrentTime;
        //Serial.print("WLtimer = "); //Serial.print(WLtimer); //Serial.print("\t");
      }

      if (WLtimer >= 5)
      {
        drainValveOff();
        waterValveOff(); //Turn off water valve
        washWaterLevelRequirementMet = 1; //water level requirement is met
        ignoreWaterLevel = 1;
        serialPrints(); Serial.println("Filled water\t");
        //lcd.setCursor(0, 1);
        //lcd.print("Filled      ");
      }
    }
  
    if (previousActualWaterLevel != currentActualWaterLevel)
    {
      CalibWLtimer=0;
      previousActualWaterLevel = currentActualWaterLevel;
      
    }
    if (previousActualWaterLevel == currentActualWaterLevel) // if water in drum is staying the same for a while
    {
      if ((CalibWLcurrentTime - CalibWLpreviousTime) >= 1000)
      {
        CalibWLtimer = CalibWLtimer + 1;
        CalibWLpreviousTime = CalibWLcurrentTime;
      }
     }
     
      if (CalibWLtimer >= 180)
      {
         pressureSensorCalibrationTop(); 
      }

  }
}

void spinWaterLevelMonitor() {
  WLcurrentTime = millis();
  CalibWLcurrentTime = millis();
  requiredWaterLevel = 5;
  pressureSensorRead();
  currentActualWaterLevel = actualWaterLevel;
//  Serial.print("requiredWaterLevel="); Serial.print(requiredWaterLevel); Serial.print("\t");
//  Serial.print("actualWaterLevel="); Serial.print(actualWaterLevel); Serial.print("\t");
//
////  Serial.print("currentActualWaterLevel="); Serial.print(currentActualWaterLevel); Serial.print("\t");
////  Serial.print("previousActualWaterLevel="); Serial.print(previousActualWaterLevel); Serial.print("\t");
//  Serial.print("CalibWLtimer="); Serial.print(CalibWLtimer); Serial.print("\t");

  if (ignoreWaterLevel == 0)
  {
    if (actualWaterLevel >= requiredWaterLevel) // if water in drum is more than selected level
    {
      WLtimer = 0;
      drainValveOn(); //Turn on drain valve
      stopMotor();
      spinWaterLevelRequirementMet = 0; //water level requirement is not met
//      Serial.print("draining water\t");
      //lcd.setCursor(0, 1);
      //lcd.print("draining     ");
    }

    if (actualWaterLevel <= requiredWaterLevel) // if water in drum is less than selected level
    {
      if ((WLcurrentTime - WLpreviousTime) >= 1000)
      {
        WLtimer = WLtimer + 1;
        WLpreviousTime = WLcurrentTime;
        //Serial.print("WLtimer = "); //Serial.print(WLtimer); //Serial.print("\t");
      }

      if (WLtimer == 4)
      {
        pressureSensorCalibrationBottom();
      }
      
      if (WLtimer >= 5)
      {
        drainValveOn();
        spinWaterLevelRequirementMet = 1; //water level requirement is met
        ignoreWaterLevel = 1;
        serialPrints(); Serial.print("drained water\t");
        //lcd.setCursor(0, 1);
        //lcd.print("drained      ");
      }
    }


    if (previousActualWaterLevel != currentActualWaterLevel)
    {
      CalibWLtimer=0;
      previousActualWaterLevel = currentActualWaterLevel;
      
    }
    if (previousActualWaterLevel == currentActualWaterLevel) // if water in drum is staying the same for a while
    {
      if ((CalibWLcurrentTime - CalibWLpreviousTime) >= 1000)
      {
        CalibWLtimer = CalibWLtimer + 1;
        CalibWLpreviousTime = CalibWLcurrentTime;
        //Serial.print("WLtimer = "); //Serial.print(WLtimer); //Serial.print("\t");
      }
     }
     
      if (CalibWLtimer >= 60)
      {
         pressureSensorCalibrationBottom(); 
      }      
      
    }
  
  
}


// Motor activity
void motorNormalSpin() {
  drySpinState = 0;
//  Serial.print("MotorSpinStateChangeTime = "); Serial.print(MotorSpinStateChangeTime); Serial.print("\t"); 

    if (WashNormalSpinState == 0) //switching cw
  {
    stopMotor(); //Serial.print("waitForSoakTime = "); Serial.print(waitForSoakTime); Serial.print("\t"); 
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( soakTimeLeft <= 0) {
      MotorSpinStateChangeTime =  timer; 
      WashNormalSpinState = 1;
      eepromWrite();
    }
  }

  if (WashNormalSpinState == 1) //switching cw
  {
    stopMotor();
    cw();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 1) {
      MotorSpinStateChangeTime =  timer; 
      WashNormalSpinState = 2;
    }
  }
  
  if (WashNormalSpinState == 2) //running
  {
    startMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t");   //Serial.print ("Motor spinning cw"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashNormalSpinTime) {
      MotorSpinStateChangeTime =  timer;
      WashNormalSpinState = 3;
    }
  }
  
  if (WashNormalSpinState == 3) //stopped
  {
    stopMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashNormalStopTime) {
      MotorSpinStateChangeTime =  timer; 
      WashNormalSpinState = 4;
    }
  }

  if (WashNormalSpinState == 4) //switching cw
  {
    stopMotor(); 
    ccw();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 1) {
      MotorSpinStateChangeTime =  timer; 
      WashNormalSpinState = 5;
    }
  }
  
  if (WashNormalSpinState == 5) //running
  {
    startMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t");   //Serial.print ("Motor spinning cw"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashNormalSpinTime) {
      MotorSpinStateChangeTime =  timer;
      WashNormalSpinState = 6;
    }
  }

    if (WashNormalSpinState == 6) //stopped
  {
    stopMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashNormalStopTime) {
      MotorSpinStateChangeTime =  timer; 
      WashNormalSpinState = 1;
    }
  }

}

void motorRapidSpin() {


  drySpinState = 0;
  if (WashRapidSpinState == 0) //stopped
  {
    stopMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t"); //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashRapidStopTime) {
      MotorSpinStateChangeTime =  timer;
      WashRapidSpinState = 1;
    }
  }
  if (WashRapidSpinState == 1) //rotating cw
  {
    cw();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t");   //Serial.print ("Motor rapid spinning cw"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashRapidSpinTime) {
      MotorSpinStateChangeTime =  timer;
      WashRapidSpinState = 2;
    }
  }
  if (WashRapidSpinState == 2) //stopped
  {
    stopMotor();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t");   //Serial.print ("Motor stopped"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashRapidStopTime) {
      MotorSpinStateChangeTime =  timer;
      WashRapidSpinState = 3;
    }
  }
  if (WashRapidSpinState == 3) //rotating ccw
  {
    ccw();
    //Serial.print ("Change time= "); //Serial.print(MotorSpinStateChangeTime);  //Serial.print ("\t"); //Serial.print ("Current time= "); //Serial.print( timer); //Serial.print ("\t");   //Serial.print ("Motor rapid spinning ccw"); //Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > WashRapidSpinTime) {
      MotorSpinStateChangeTime =  timer;
      WashRapidSpinState = 0;
    }
  }
}

void motorDrySpin() {

  WashNormalSpinState = 0;
  WashRapidSpinState = 0;
//  Serial.print ("drySpinState"); Serial.print(drySpinState); Serial.print ("\t");

  if (drySpinState == 0) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for 1 sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 1) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 1;
    }
  }

  if (drySpinState == 1) //stopped
  {
    stopMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t"); Serial.print ("Motor stopped"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 8) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 2;
    }
  }

  if (drySpinState == 2) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for 1 sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 1) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 3;
    }
  }

  if (drySpinState == 3) //stopped
  {
    stopMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("Motor stopped"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 2) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 4;
    }
  }

  if (drySpinState == 4) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for 3 sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 2) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 5;
    }
  }

  if (drySpinState == 5) //stopped
  {
    stopMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("Motor stopped"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 2) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 6;
    }
  }

  if (drySpinState == 6) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for 3 sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 3) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 7;
    }
  }

  if (drySpinState == 7) //stopped
  {
    stopMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("Motor stopped"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 3) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 8;
    }
  }

  if (drySpinState == 8) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for 3 sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 5) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 9;
    }
  }

  if (drySpinState == 9) //stopped
  {
    stopMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("Motor stopped"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 3) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 10;
    }
  }

  if (drySpinState == 10) //rotating cw
  {
    cw();
    startMotor();
//    Serial.print ("Change time= "); Serial.print(MotorSpinStateChangeTime);  Serial.print ("\t"); Serial.print ("Current time= "); Serial.print( timer); Serial.print ("\t");   Serial.print ("spin for x sec"); Serial.print ("\t");
    if ( timer - MotorSpinStateChangeTime > 900) {
      MotorSpinStateChangeTime =  timer;
      drySpinState = 0;
    }
  }
}


//Relay activities
void cw() {
  digitalWrite(motorDirectionPin, LOW);
  //serialPrints() Serial.print("cw"); Serial.print("\n");
}

void ccw() {
  digitalWrite(motorDirectionPin, HIGH);
  //serialPrints() Serial.print("ccw"); Serial.print("\n");
}

void stopMotor() {
  digitalWrite(motorOnOffPin, LOW);
  if (play == 1){
  // serialPrints() Serial.print("motor stopped"); Serial.print("\n");
  }
}

void startMotor() {
  digitalWrite(motorOnOffPin, HIGH);
   //serialPrints() Serial.print("motor started"); Serial.print("\n");
}

void drainValveOn() {
  digitalWrite(waterInletValvePin, LOW);
  digitalWrite(drainValvePin, HIGH); //Turn on drain valve
  digitalWrite(motorDirectionPin, LOW);
  //Serial.println("drain on");
}

void drainValveOff() {
  digitalWrite(drainValvePin, LOW); //Turn off drain valve
  //Serial.println("drain off");
}

void waterValveOn() {
  digitalWrite(waterInletValvePin, HIGH);
  digitalWrite(drainValvePin, LOW);
  digitalWrite(motorOnOffPin, LOW);
  digitalWrite(motorDirectionPin, LOW);
  //Serial.println("water on");
}

void waterValveOff() {
  digitalWrite(waterInletValvePin, LOW);
  //Serial.println("water off");
}


// Modes
void NormalMode() {
  WashCycleCount = 3;
  waitForSoakTime = 0;
  WashCycleTime =  (600); //15 min 36
  SpinCycleTime = (180); //5 min



  if (modeAccomplished == 0)
  {
    if (completedWashCycleCount <= WashCycleCount)
    {
//      Serial.print ("completedWashCycleCount = "); Serial.print (completedWashCycleCount);Serial.print ("\t");
      if (spinRequest == 0)
      {
        washWaterLevelMonitor();
        if (ignoreWaterLevel == 0) {
          stopMotor();
        }
        //
        if (ignoreWaterLevel == 1)
        {
          if (washModeStarted == 0) {
            washModeStartTime =  timer;
            washModeStarted = 1;
            EEPROM.write(10, 1);
            EEPROM.write(11, 0);
          }
          if (washModeStarted == 1) {
            washModeTimeLeft = (WashCycleTime - (timer - washModeStartTime));
            if (washModeTimeLeft > 0)
            {
              //lcd.setCursor(0, 1);
              //lcd.print("Washing      ");
              motorNormalSpin();//start wash cycle

            }
            if ( washModeTimeLeft <= 0)
            {
              serialPrints(); Serial.print("Wash cycle completed\n");
              stopMotor();
              spinRequest = 1;
              spinModeTimeLeft = 0;
              spinModeStarted = 0;
              ignoreWaterLevel = 0;
              WashNormalSpinState = 1;
              timer = 0;
              MotorSpinStateChangeTime=0;
              eepromWrite();
              //start spin cycle
            }

//            Serial.print("WashModeTimeLeft="); Serial.print(washModeTimeLeft); Serial.print("\t"); //Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\t"); 
          }
        }
      }

      if (spinRequest == 1)
      {
        EEPROM.write(6, spinRequest);
        spinWaterLevelMonitor();
        if (spinWaterLevelRequirementMet == 0) {
          stopMotor();
        }
        //

        if (spinWaterLevelRequirementMet == 1)
        {
          if (spinModeStarted == 0) {
            spinModeStartTime =  timer;
            spinModeStarted = 1;
            EEPROM.write(10, 0);
            EEPROM.write(11, 1);
          }
          if (spinModeStarted == 1) {
            spinModeTimeLeft = (SpinCycleTime - (timer - spinModeStartTime));
            if (spinModeTimeLeft > 0)
            {
              //lcd.setCursor(0, 1);
              //lcd.print("Spinning     ");
              motorDrySpin(); //start spin cycle
            }
            if ( spinModeTimeLeft <= 0)
            {
              stopMotor();
              drainValveOff();
              delay (9000);
              pressureSensorCalibrationBottom();
              delay (1000);
              {
              //Serial.print("spin cycle completed\t");
              completedWashCycleCount = completedWashCycleCount + 1;
              spinRequest = 0;
              ignoreWaterLevel = 0;
              EEPROM.write(6, spinRequest);
              washModeTimeLeft = 0;
              washModeStarted = 0;
              MotorSpinStateChangeTime=0;
              timer = 0;
              eepromWrite();
              //start spin cycle
              }
            }
//            Serial.print("spinModeTimeLeft="); Serial.print(spinModeTimeLeft); Serial.print("\t");
          }
        }
      }
    
}

    if (completedWashCycleCount >= WashCycleCount)
    {
      modeAccomplished = 1;
    }
  }

  if (modeAccomplished == 1)
  {
    eepromWrite();
    washModeStartTime = 0;
    spinModeStartTime = 0;
    completedWashCycleCount = 0;
    WashNormalSpinState = 0;
    MotorSpinStateChangeTime=0;

    play = 0;
    interruptedCount = 0;
    timer = 0;

  }
}

void HeavyMode() {
  WashCycleCount = 3;
  if (completedWashCycleCount >  0){ waitForSoakTime = 0; }
  WashCycleTime =  (720); //15 min 72
  SpinCycleTime = (210); //5 min



  if (modeAccomplished == 0)
  {
    if (completedWashCycleCount <= WashCycleCount)
    {
//            Serial.print ("completedWashCycleCount = "); Serial.print (completedWashCycleCount);Serial.print ("\t");
      if (spinRequest == 0)
      {
        washWaterLevelMonitor();
        if (ignoreWaterLevel == 0) {
          soakTimeLeft = waitForSoakTime;
          stopMotor();
        }
        //
        if (ignoreWaterLevel == 1)
        {
          if (washModeStarted == 0 && soakTimeLeft >0)
          {
            isSoaking = 1;
            currentSoakTime= timer; 
            if (currentSoakTime - previousSoakTime >= 1)
              {
                soakTimeLeft = soakTimeLeft - 1;
                previousSoakTime = currentSoakTime;
              }
           }
          if (washModeStarted == 0 && soakTimeLeft <=0) {
            isSoaking = 0;
            washModeStartTime =  timer;
            washModeStarted = 1;
            EEPROM.write(10, 1);
            EEPROM.write(11, 0);
          }
          if (washModeStarted == 1) {
            soakTimeLeft = 0;
            washModeTimeLeft = (WashCycleTime - (timer - washModeStartTime));
            if (washModeTimeLeft > 0)
            {
              //lcd.setCursor(0, 1);
              //lcd.print("Washing      ");
              motorNormalSpin();//start wash cycle

            }
            if ( washModeTimeLeft <= 0)
            {
              serialPrints(); Serial.print("Wash cycle completed\n");
              stopMotor();
              spinRequest = 1;
              spinModeTimeLeft = 0;
              spinModeStarted = 0;
              ignoreWaterLevel = 0;
              WashNormalSpinState = 1;
              timer = 0;
              MotorSpinStateChangeTime=0;
              eepromWrite();
              //start spin cycle
            }

//            Serial.print("WashModeTimeLeft="); Serial.print(washModeTimeLeft); Serial.print("\t"); // Serial.print("WashNormalSpinState="); Serial.print(WashNormalSpinState); Serial.print("\t");   
          }
        }
      }

      if (spinRequest == 1)
      {
        EEPROM.write(6, spinRequest);
        spinWaterLevelMonitor();
        if (spinWaterLevelRequirementMet == 0) {
          stopMotor();
        }
        //

        if (spinWaterLevelRequirementMet == 1)
        {
          if (spinModeStarted == 0) {
            spinModeStartTime =  timer;
            spinModeStarted = 1;
            EEPROM.write(10, 0);
            EEPROM.write(11, 1);
          }
          if (spinModeStarted == 1) {
            spinModeTimeLeft = (SpinCycleTime - (timer - spinModeStartTime));
            if (spinModeTimeLeft > 0)
            {
              //lcd.setCursor(0, 1);
              //lcd.print("Spinning     ");
              motorDrySpin(); //start spin cycle
            }
            if ( spinModeTimeLeft <= 0)
            {
              stopMotor();
              drainValveOff();
              delay (9000);
              pressureSensorCalibrationBottom();
              delay (1000);
              {
              //Serial.print("spin cycle completed\t");
              completedWashCycleCount = completedWashCycleCount + 1;
              spinRequest = 0;
              ignoreWaterLevel = 0;
              EEPROM.write(6, spinRequest);
              washModeTimeLeft = 0;
              washModeStarted = 0;
              timer = 0;
              MotorSpinStateChangeTime=0;
              eepromWrite();
              }
              //start spin cycle
            }
//            Serial.print("spinModeTimeLeft="); Serial.print(spinModeTimeLeft); Serial.print("\t");
          }
        }
      }
    }

    if (completedWashCycleCount >= WashCycleCount)
    {
      modeAccomplished = 1;
    }
  }

  if (modeAccomplished == 1)
  {
    eepromWrite();
    washModeStartTime = 0;
    spinModeStartTime = 0;
    completedWashCycleCount = 0;
    WashNormalSpinState = 0;
    MotorSpinStateChangeTime=0;

    play = 0;
    interruptedCount = 0;
    timer = 0;

  }
}

void DryMode() {
//  WashCycleCount = 1;
  WashCycleTime = (0); //15 min
  SpinCycleTime = (210); //5 min210
  spinRequest = 1;


  if (modeAccomplished == 0)
  {
    if (completedWashCycleCount <= WashCycleCount)
    {
//            Serial.print ("completedWashCycleCount = "); Serial.print (completedWashCycleCount);Serial.print ("\t");

      if (spinRequest == 1)
      {
        EEPROM.write(6, spinRequest);
        spinWaterLevelMonitor();
        if (spinWaterLevelRequirementMet == 0) {
          stopMotor();
        }
        //

        if (spinWaterLevelRequirementMet == 1)
        {
          if (spinModeStarted == 0) {
            spinModeStartTime =  timer;
            spinModeStarted = 1;
            EEPROM.write(10, 0);
            EEPROM.write(11, 1);
          }
          if (spinModeStarted == 1) {
            spinModeTimeLeft = (SpinCycleTime - (timer - spinModeStartTime));
            if (spinModeTimeLeft > 0)
            {
              //lcd.setCursor(0, 1);
              //lcd.print("Spinning     ");
              motorDrySpin(); //start spin cycle
            }
            if ( spinModeTimeLeft <= 0)
            {
              stopMotor();
              drainValveOff();
              //Serial.print("spin cycle completed\t");
              delay (9000);
              pressureSensorCalibrationBottom();
              delay (1000);
              {
              completedWashCycleCount = completedWashCycleCount + 1;
              spinRequest = 0;
              ignoreWaterLevel = 0;
              EEPROM.write(6, spinRequest);
              washModeTimeLeft = 0;
              washModeStarted = 0;
              MotorSpinStateChangeTime=0;
              timer = 0;
              eepromWrite();
              //start spin cycle
              }
            }
//            Serial.print("spinModeTimeLeft="); Serial.print(spinModeTimeLeft); Serial.print("\t");
          }
        }
      }
  
    }

    if (completedWashCycleCount >= WashCycleCount)
    {
      modeAccomplished = 1;
    }
  }

  if (modeAccomplished == 1)
  {
    eepromWrite();
    washModeStartTime = 0;
    spinModeStartTime = 0;
    completedWashCycleCount = 0;
    WashNormalSpinState = 0;
    MotorSpinStateChangeTime=0;

    play = 0;
    interruptedCount = 0;
    timer = 0;

  }
}
