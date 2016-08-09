//arduinix
//Main controller for BrewMaster
//Handles gas burner control and saftey, kettle temperature monitoring

//import libraries
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MAX6675.h>

//decalare variables for flame control pins
int flameReqPin = 34;
int flameOnTestPin = 32;  //pin for a test switch to tell the controller that the flame is lit
int flameIndLedPin = 38;  //flame lit indicator led
int flameFltLedPin = 36;  //flame fault indicator led
int gasValRlyPin = 52;    //pin connected to the propane control valve relay
int ignRlyPin = 50;       //pin connected to the flame igniter relay
int kettleThermistorPin = 2;    //kettle temperature sensor thermistor
int thermoCLK = 3;        //SCK pin of MAX6675 for flame temperature reading
int thermoDO = 4;         //CS pin on MAX6675 for flame temperature reading
int thermoCS = 5;         //SO pin of MAX6675 for flame temperature reading
int optFlmSenPin = 0;     //analog pin to which optical flame sensor is connected, here we set it to 0 so it means A0



//intitialize variables for burner control
boolean burnerRequested = false;    //status of the flame requested call
boolean burnerFault = false;        //status of the burner fault saftey control
boolean attemptingIgnition = false; //current stage of the burner ignition process
boolean flameDetected = false;      //flame detection status for ignition timing control
boolean flameLit = false;           //status of the flame, set to true if the flame currently lit
boolean burnerAutoReq = false;      //autorequest mode of the burner, set to true if the controller is in mode 1 (auto) and being controled by the set temperature
int maxFailCount = 3;               //maximum number of times that controller is allowed to attempt ignition before going into failure mode
int burnerFailCount = 0;            //accumulator to store the amount of failed ignition attempts when this variable is >= to maxFailCount burnerFault is set to true
int flameRelayFaults = 0;           //accumulates the number of cycles during which the flame relay pin is off and there is still a flame detected, indicates a failed sensor
long maxIgnitionTime = 5000;        //the allowable time in milliseconds to attempt a successful ignition during each ignition attempt cycle
long ignitionTime = 0;              //the time taken to successfully light the burner
long burnerPoleTime = 500;          //time in milliseconds to poll for changes to the burner request status or flame status
long lastBurnerPoleTime = 0;        //time since the last burner poll time

//varables for the optical flame sensor
int optFlmSenReading = 0;          //voltage reading of the optical flame sensor pin
int specOptFlmSenReading = 20;     //the optFlmSenPin flame sensor pin must exceed this value before it is determined that the flame is lit

//variables for temperatrure control
long lastTempPrintTime = 0;      //lcd last print time
long tempPrintTime = 500;        //milliseconds to wait in between temp print outs to the LCD
long tempPoleTime = 500;         //amount of time to wait between temperature sensor poles, more time, less gitter
long lastTempPoleTime = 0;       //the last pole time of the temperature sensor
float temp = 0;                  //temperature reading from the thermistor 
byte kettleTemp = 0;             //the temperature of the kettle contents in *f
byte setTemp = 0;                //the desired temperature of the kettle set on the control interface
byte mode = 0;                   //the operating mode of the system, set on the control interface
int token = 1;                   //i2c token for sending/reciving data (not yet implemented)                                      
float pad = 9850;                // balance/pad resistor value                                    
float thermr = 10000;            // moninal resistance of the thermistor use as the kettle temperature sensor

//variables for LCD Screen
long lastLCDPrintTime = 0;      //the last time the lcd was refreshed
long LCDPrintTime = 1000;       //amount of time in milliseconds to wait on each lcd refresh

//variables for display mode
int dispMode = 1;               //the current mode of the display (moved to the display controller in this version)
int numDisp = 5;                //number of available displays
long debounce = 200;            //milliseconds that button must be held for to change the lcd

//define the LCD pins
#define I2C_ADDR    0x3F  // Define I2C Address where the PCF8574A is (moved to control panel controller in this version
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

//initialize the MAX6675 sensor
//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//set the LCD address for proper display
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

void setup()
{ 

  //intitialize the input pins
  pinMode(flameReqPin, INPUT);
  pinMode(flameOnTestPin, INPUT);
  
  //default the pins to high to keep burner off
  digitalWrite(gasValRlyPin, HIGH);
  digitalWrite(ignRlyPin, HIGH);
 
  //initialize the output pins
  pinMode(flameIndLedPin, OUTPUT);
  pinMode(flameFltLedPin, OUTPUT);
  pinMode(gasValRlyPin, OUTPUT);
  pinMode(ignRlyPin, OUTPUT);
  
  //start the serial port
  Serial.begin(9600);          //start the serial monitor with 9600 baud
  
  //join the i2c bus as controller number 1
  Wire.begin(1); // join i2c bus (address optional for master
  Wire.onReceive(receiveEvent); // register event
  
  //set up the lcd screen
  lcd.begin(20, 4);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE); // init the backlight
  lcd.setBacklight(10);
  lcd.setCursor(0,0);
  lcd.print("BrewMaster");
  lcd.setCursor(0,1);
  lcd.print("V1.0(beta)");
  lcd.setCursor(0,2);
  lcd.print("Happy Brewing!");
  delay (3000);
  lcd.clear();
  lcd.setCursor(10,0);
  lcd.print("|");
    
}

void loop()
{ 
    //check the value of the light sensor
    optFlmSenReading = analogRead(optFlmSenPin);      //reads the ldr’s value through optFlmSenPin
    
    //turn on flame lit led if the optFlmSenPin Value is above the specified limit
    if(optFlmSenReading > specOptFlmSenReading)
    {
      //turn on flame lit led
      digitalWrite(flameIndLedPin, HIGH);
    }
    else
    {
      //turn off flame lit led
      digitalWrite(flameIndLedPin, LOW);
    }    
  
  //Read the temp from the MAX6675
      if((millis() - tempPoleTime) > lastTempPoleTime)
      {  
           // basic readout test, just print the current temp
          Serial.print("C = "); 
          //Serial.println(thermocouple.readCelsius());
          Serial.print("F = ");
          //Serial.println(thermocouple.readFahrenheit());
          
          //store the temperatue in a varable
          //kettleTemp = (thermocouple.readFahrenheit());
        
          //set lastTempPoleTime to the current value of millis
          lastTempPoleTime = millis();
      }
      
      //read the kettle temperature using a thermistor
      temp=Thermistor(analogRead(kettleThermistorPin));       // read ADC and  convert it to Celsius
      //Serial.print("Celsius: ");
      //Serial.print(temp,1);                             // display Celsius
      temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
      //Serial.print(", Fahrenheit: ");
      //Serial.print(temp,1);                             // display  Fahrenheit
      //Serial.println("");                                  
      kettleTemp = temp;
      
     //transmit data to the other contoller
     transmitEvent();
     
     //determine the mode and if the flame is needed
     if (mode == 1)
     {
         burnerAutoReq = true;
     }
     else if (mode == 2)
     {
       //check the temperature and determine to call for burner or not
       if (kettleTemp <= setTemp)
       {
         burnerAutoReq = true;
       }
       else if (kettleTemp > (setTemp + 2))
       {
          burnerAutoReq = false; 
       }
     }
     else
     {
         burnerAutoReq = false; 
     }
    
    //print the temperature if there is an lcd attached 
    if((millis() - lastTempPrintTime) > tempPrintTime)
    {
        //print the optFlmSenReading on the serial port
        Serial.println(optFlmSenReading);       //prints the optFlmSenPin values to serial monitor
        lastTempPrintTime = millis();
    }
    
    //update the lcd display if the appropriate amount of time has past
    if((millis() - lastLCDPrintTime) > LCDPrintTime)
    {
        //switch the display based on the current display mode
        switch (dispMode)
        {
          //display mode 1
          case 1:
            //call function to clear the lcd screen
            //wipeLines()
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("ST:");
            lcd.setCursor(3,0);
            //lcd.print(desiredTemp);
            lcd.setCursor(8,0);
            lcd.print("CT:");
            lcd.setCursor(11,0);
            lcd.print(kettleTemp);
            
            lcd.setCursor(0,1);
            lcd.print("Burner: ");
            lcd.setCursor(8,1);
            //print the burner status
            if(flameLit)
            {
                //print "On"
                lcd.print("On");
            }
            else if(!flameLit)
            {
                //print "Off"
                lcd.print("Off");
                
                 if(burnerFault)
                    {
                        //print "Fault"
                        lcd.print("Fault!");
                    }
              
            }    
          break;
          
          //display mode 2
          case 2:
          
          break;
          
          //display mode 3
          case 3:
          
          break;
          
          //display mode 4
          case 4:
          
          break;
          
          //display mode 5
          case 5:
          
          break;
          
        } //end of switch statement
        
        
        
        //update the lastLCDPrintTime variable to the current value of millis
        lastLCDPrintTime = millis();
    }

    // steps to light burner, only perform them every n milliseconds
    if((millis() - lastBurnerPoleTime) > burnerPoleTime)
    {
        //Check the flame request pin and set the burner request variable accordingly
      if (digitalRead(flameReqPin) == HIGH || burnerAutoReq)
      {
        //set the flame requested variable to true
        burnerRequested = true;
      }
      else
      {
        //set the flame requested variable to false
        burnerRequested = false;
      }
      
      //check the status of of the flame sensor pin
      if(digitalRead(flameOnTestPin) == HIGH || optFlmSenReading > specOptFlmSenReading)
      {
        flameDetected = true;
      }
      else
      {
        flameDetected = false;
      }
      
      //illuminate the burner fault light if the burner fault indicator variable is true
      if(burnerFault)
      {
        //illuminlate burnerfault led
        digitalWrite(flameFltLedPin, HIGH);
      }
      else
      {
        //turn off burner fault led
        digitalWrite(flameFltLedPin, LOW);
      }
      
      //change the state of the flameLit variable
      if(flameDetected && !attemptingIgnition && !burnerFault)
      {
        //set the flameLit variable to true
        flameLit = true;
      }
      else
      {
        //set the flameLit variable to false
        flameLit = false;
      }
      
      //set a flame fault if the flame sensor is showing flame and the gas valve is off
      if(flameDetected && (digitalRead(gasValRlyPin)) == HIGH)
      {
        //increment the flameRelayFaults variable by 1
        flameRelayFaults ++;
      }
      
      if (burnerRequested) //we want this to keep looping in case there is a problem with the burner
      {
        //call funcion to light burner
        lightBurner();
      }
      else
      {
        //call function to extinguish burner
        extinguishBurner();
        
        //clear burner fault
        burnerFault = false;
      }
      
      //set error if flame is indicated with closed gas valve
      if(flameRelayFaults > 3)
      {
        //set the burnerFault variable to truee
        burnerFault = true;
        
         //set the flameRelayFaults variable to 0
        flameRelayFaults = 0;
      }
      
      //set the last burner pole time to the current chip time
      lastBurnerPoleTime = millis();
      
    } //end of burner pole time if statement
  //else for this would be to shut off igniter leave gas on set flame var to on clear error counter
  
  
  /*
  //reading display switch button and chageing variable
  btnDReading = digitalRead(btnDPin);
  if (btnDReading == HIGH && millis() - time > debounce)
  {
    if (dispCount < numDisp)
    {
      dispCount ++;
    }
    else
   {
      dispCount = 1; 
    }
    time = millis();    
  }
    
  //refreash the display only if the correct amount of time has elapsed.
  if ((millis() - bRefTime) > bRefRate)
  {
    bRefTime = millis();
    
 */
  
}


//function to light the burner
void lightBurner()
{
  //set the burner fault variable to true if the fail count is higher than the max
  if(burnerFailCount >= maxFailCount)
  {
     //set the burnerFault variable to truee
    burnerFault = true;
    
    //set the burner fail count back to 0
    burnerFailCount = 0;
  }
  
  //turn on the igniter and open gas valve if conditions are met
   if(burnerRequested && !burnerFault)
    {
      //attempt to light if there is no flame detected
      if(!flameLit && !attemptingIgnition)
      {
      //set the igniter relay pin to low
      digitalWrite(ignRlyPin, LOW);
      
      //set the gas valve relay pin to low
      digitalWrite(gasValRlyPin, LOW);
      
      //set the attempting ignition variable to true
      attemptingIgnition = true;
      
      ignitionTime = millis();
      }
    }
    
    //close the gas valve and turn off the igniter if the burner is no longer requested or fault occured
    else
    {
      //call function to extinguish burner
      extinguishBurner();
      
      //set the attempting ignition variable to false
      attemptingIgnition = false;
    }
  
  //add outer if statement for flame detection and move attempting ignition to here
  if(!flameDetected)
  {
  
    if(attemptingIgnition && (millis() - ignitionTime) > maxIgnitionTime)
    {
      //call function to extinguish the burner
      extinguishBurner();
      
      //increment the burner fail count by 1
      burnerFailCount ++;
    }
  }
  else //this condition is true if a flame is detected
  {
    //shut off the igniter and leave gas on
    digitalWrite(ignRlyPin, HIGH);
    
    //stop attempting ignition
    attemptingIgnition = false;
    
    //set the burner fail count to 0
    burnerFailCount = 0;
    
  }

}

//function to extinguish the burner
void extinguishBurner()
{
  //set the igniter relay pin to high
  digitalWrite(ignRlyPin, HIGH);
      
  //set the gas valve relay pin to high
  digitalWrite(gasValRlyPin, HIGH);
  
  //attempting ignition false
  attemptingIgnition = false;
}

//function to transmit events
void transmitEvent() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(kettleTemp);              // sends one byte
  Wire.write(setTemp);
  Wire.write(mode);
  //send the token back
  Wire.write(token);
  token = 0;
  Wire.endTransmission();    // stop transmitting
}
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int numEventsRx) {
  while (1 < Wire.available()) { // loop through all but the last
    setTemp = Wire.read();
    mode = Wire.read();
  }
   //get the token to send
   token = Wire.read();
}

//function to check the temperature
float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.

  Resistance=pad*((1024.0 / RawADC) - 1);
  Temp = log(Resistance); //store the log of the resistance so that it does not need to be calculated later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  //Convert Kelvin to Celsius                      
  return Temp;           //Return the Temperature
}
