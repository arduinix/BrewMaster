//arduinix
//Display controller for BrewMaster
//Handles display and human interface

//import libraries
#include <Wire.h>
#include <LiquidCrystal.h>


byte lcdNumCols = 20; //number of columns in the LCD
byte lcdNumRows = 4; //number of rows in the LCD

//declare vars for displaybutton control
byte btnDPin = 6;   //display mode select pin
byte btnMode = 7;   //mode select pin
byte rECLK = 8; //rotary encoder clk pin
byte rEDT = 9; //rotary encoder dt pin
int btnDState = HIGH;
int btnDReading;
int btnDPrevious = LOW;
long timePress = 0;
long debounce = 500; //milliseconds that button must be held for to cause action.
long tempDebounce = 100;
int value = 0;
int buttonState = 0;
int numDisp = 4;
int numModes = 2;
int currentDisp = 1;
int lastDisp = 0;
int test = 0;

//rotary encoder variables
int pinALast;
int aVal;
int encoderPosCount = 0;
int maxPos = 3000;
int minPos = 0;
int lastPosCount = 0 ;

//declare variables for stats
float kettleTemp= 0;
byte tempSet = 20;
float tempSetConf = 0;
byte modeSet = 0;
byte modeSetConf = 0; 
byte token = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);



//Create variables for storage of values

void setup()
{ 
  //get the intital value of the encoder clock pin
  pinALast = digitalRead(rECLK);
  
  //i2c bus stuff
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  
  //set up the LCD
  lcd.begin(lcdNumCols, lcdNumRows);
  lcd.setCursor(4,0);
  lcd.print("Welcome to");
  lcd.setCursor(4,1);
  lcd.print("Brew Master!");
  delay (500);
  lcd.clear();
  lcd.setCursor(10,0);
  lcd.print("|");
  
  //Init the pin for the backlight control
  //pinMode(backlightPin, OUTPUT);
  
  //Init the pin for display control
  pinMode(btnDPin, INPUT);
  
  //init the pin for mode control
  pinMode(btnMode, INPUT);
  
  //init the pin for rotary encoder
  pinMode(rECLK, INPUT);
  
  //init the pin for rotary encoder
  pinMode(rEDT , INPUT);
  
}

void loop()
{
  //send data to main controller
  transmitEvent();
  
  //reading display switch button and chageing variable
  if (digitalRead(btnDPin) == HIGH && millis() - timePress > debounce)
  {
    if (currentDisp < numDisp)
    {
      currentDisp ++;
    }
    else
   {
      currentDisp = 1; 
   }
    timePress = millis();
    lcd.clear();
  }
  
  //temperature control
  /*
  //reading for increase button state
  if (digitalRead(btnInc) == HIGH && millis() - timePress > tempDebounce)
  {
    //if we are in the temperature display increase the set temp
    if (currentDisp == 1)
    {
      tempSet ++;
    }
    timePress = millis();
  }
  
  //reading for decrease button state
  if (digitalRead(btnDec) == HIGH && millis() - timePress > tempDebounce)
  {
    //if we are in the temperature display, decrease the temp
    if (currentDisp == 1)
    {
      tempSet --;
    }
    timePress = millis();
  }
 */
 //read the value from the rotary encoder pins and increment or decrement
 aVal = digitalRead(rECLK);
  if (aVal != pinALast)
  { // Means the knob is rotating
    // if the knob is rotating, we need to determine direction
    // We do that by reading pin B.
    if (digitalRead(rEDT) != aVal)
    {  // Means pin A Changed first - We're Rotating Clockwise
       //increase the value
       if (digitalRead(rEDT) != aVal) {  // Means pin A Changed first - We're Rotating Clockwise
          if(encoderPosCount < maxPos)
          {
            encoderPosCount +=1;
          }
    }
    else
    {// Otherwise B changed first and we're moving CCW
      //decrease the value
       if (encoderPosCount > minPos)
        {
          encoderPosCount -=1;
        }
      
      }  
    }
  }
 
 
//if in display mode

//encoder is currently not working
if (currentDisp == 1)
    {
      //increase the temperature value if the rotary encoder has moved ccw
      if (encoderPosCount > lastPosCount) 
      {
        //increase the temperature
        tempSet ++;
      }
      
      else if (encoderPosCount < lastPosCount)
      {
         //decrease the temperature
         tempSet --; 
      }
 
    }
  lastPosCount = encoderPosCount;
 
 
 
 
 
 
 
  
  //read the state of the mode button and change the mode
  if (digitalRead(btnMode) == HIGH && millis() - timePress > debounce)
  {
    if (modeSet < numModes)
    {
      modeSet ++;
    }
    else
   {
      modeSet = 0; 
   }
    timePress = millis();
  }
  
  
  //Set variables for storage of values. Some values will be read from the master controller.
  //Other values will be queried from the master controller
  //This will be accomplished via the wire library or by UART.

switch (currentDisp)
{
  case 1:
  lcd.setCursor(0,0);
  lcd.print("Brew Stats Main");
  lcd.setCursor(0,1);
  lcd.print("Kettle Temp:");
  lcd.setCursor(12,1);
  lcd.print(kettleTemp);
  lcd.setCursor(0,2);
  lcd.print("Set Temp * :");
  lcd.setCursor(12,2);
  lcd.print(tempSetConf);
  lcd.setCursor(0,3);
  lcd.print("Mode:");//brew, cool, manual, auto
  lcd.setCursor(5,3);
  lcd.print(modeSetConf);
  
  
    break;
    
  case 2:
  lcd.setCursor(0,0);
  lcd.print("Burner Performance 1");
  lcd.setCursor(0,1);
  lcd.print("Ign Faults   :");
  lcd.setCursor(0,2);
  lcd.print("Flame Eye val:");
  lcd.setCursor(0,3);
  lcd.print("Avg Ign Time :");
    break;
    
  case 3:
  lcd.setCursor(0,0);
  lcd.print("Burner Performance 2");
  lcd.setCursor(0,1);
  lcd.print("Flame on Time:");
  lcd.setCursor(0,2);
  lcd.print("Flame Temp   :");
  lcd.setCursor(0,3);
  lcd.print("Gas Pressure :");
  
    break;
    
  case 4:
  lcd.setCursor(0,0);
  lcd.print("Cooling Performance");
  lcd.setCursor(0,1);
  lcd.print("Evap Temp   :");
  lcd.setCursor(0,2);
  lcd.print("Cond Temp   :");
  lcd.setCursor(0,3);
  lcd.print("Comp on time:");
  
    break;
  }
 
}


//function to transmit events
void transmitEvent() {
  Wire.beginTransmission(1); // transmit to device #8
  Wire.write(tempSet);
  Wire.write(modeSet);
  //send the token back
  Wire.write(token);
  token = 0;
  Wire.endTransmission();    // stop transmitting
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int numEventsRx) {
  while (1 < Wire.available()) { // loop through all but the last
    kettleTemp = Wire.read();    // receive byte as a float
    tempSetConf =  Wire.read();
    modeSetConf = Wire.read();
  }
  //get the token to send
  token = Wire.read();
}


