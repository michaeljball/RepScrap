/**********************************************************************************************
*  3Dprint-teensy-Demo1.ino   02-12-2015   unix_guru at hotmail.com   @unix_guru on twitter
*  http://arduino-pi.blogspot.com
*
*  This sketch allows you to run two salvaged printer carriages for X/Y axis using their 
*  linear encoder strips for tracking on a Teensy 3.1 uController module based on Freescale's
*  Kinetis K20DX256 ARM Coretex-M4 processor with two hardware based Quadrature Decoder channels
*
*  The purpose of this sketch is to test and tune the PID loops for two AXIS simultaneously
*
*  This example uses the Arduino PID Library found at:
*  https://github.com/br3ttb/Arduino-PID-Library/archive/master.zip
*
*
* The most important part of this entire project however came from Trudy Benjamin with 
* her FTM based Quadrature Decoder library.
* https://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x
*
* Thank you Trudy.
*
*
************************************************************************************************/

// wire for Teensy 3.1 as per https://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3
#include <i2c_t3.h>  
#include <PID_v1.h> 
#include "LiquidCrystal.h"

// Connect LiquidCrystal display via i2c, default address #0x20 (A0-A2 not jumpered)
LiquidCrystal lcd(0);


#include <Time.h>  //  https://github.com/PaulStoffregen/Time    Teensy RTC

time_t getTeensy3Time() { return Teensy3Clock.get(); }


#define frontstop  100            // Right most encoder boundary (allowing for some cushion)
#define backstop  7700            // Left most encoder boundary

#define XaxisPWM    23            //  PWM pin to drive X-Axis Motor
#define XaxisDir    11            //  Direction control for X-Axis
#define XaxisEND    5             //  Endstop for X-Axis (as yet unused)

#define YaxisPWM    22            //  PWM pin to drive Y-Axis Motor
#define YaxisDir    13            //  Direction control for Y-Axis
#define YaxisEND    8             //  Endstop for Y-Axis (as yet unused)

/********************************************************************************************
*  The Teensy Freescale FTM Quadrature Decoder channels are tied to specific pins 
*
*  X-Axis Quadrature Encoder Phase A is connected to Teensy 3.1 pin 3  using FTM1
*  X-Axis Quadrature Encoder Phase B is connected to Teensy 3.1 pin 4  using FTM1
*  Y-Axis Quadrature Encoder Phase A is connected to Teensy 3.1 pin 32 using FTM2
*  Y-Axis Quadrature Encoder Phase B is connected to Teensy 3.1 pin 25 using FTM2
*
**********************************************************************************************/

#define FORWARD      0
#define BACKWARD     1

#include "QuadDecode_def.h"        // Include the FlexTimer QuadDecoder library
#include <stdbool.h>


#define GUI_UPDATE_TIME	 500000	  // 500 mSec update
IntervalTimer serialTimer;	  // How often to update LCD display
void timerInt(void);	          // Main interval timing loop interrupt

#define STATE_UPDATE_TIME	 500000	  // 500 mSec update
IntervalTimer stateTimer;	  // How often to update state machine
void stateInt(void);	          // State machine timing loop interrupt


// Variables from CMM program
volatile int32_t rtX=0, rtY=0;	                // Realtime values of X,Y
volatile bool doOutput=false;                   // Run output routine
volatile bool doState=false;                    // Check next state
volatile byte currentState=0;                   // Initial Current State
volatile bool mode = 0;                         // Random or linear test
int linearStep = 2;                             // How far to travel between steps

QuadDecode<1> xPosn;	// Template using FTM1
QuadDecode<2> yPosn;	// Template using FTM2




double Spd=255, XaxisSpd,  YaxisSpd;        // Carriage speed from 0-255

/*working variables for PID routines*/
// Tuning parameters
float KpX=10,  KpY=10;                        //Initial Proportional Gain 
float KiX=50,   KiY=50;                       //Initial Integral Gain 
float KdX=0.2,  KdY=0.2;                  //Initial Differential Gain 

double XaxisSetpoint=0, XoldSetpoint=0;      // Taget position for carriage
double YaxisSetpoint=0, YoldSetpoint=0;      // Taget position for carriage
double XaxisPos=0, YaxisPos=0;	          // Realtime values of X,Y

long timeToAcquire=0;                      // How long in millis to acquire new target.

// Instantiate X and Y axis PID controls
PID XaxisPID(&XaxisPos, &XaxisSpd, &XaxisSetpoint, KpX, KiX, KdX, DIRECT); 
PID YaxisPID(&YaxisPos, &YaxisSpd, &YaxisSetpoint, KpY, KiY, KdY, DIRECT); 
const int sampleRate = 1;                 // Calling compute() from a timer interrupt.

char disbuffer[32];        // Used for formatting numbers to display


// ================================== Variables related to Command Processing ==============================================================
        
char Command = 's';
int Parameter = 0;

char inData[64];                                           // Buffer for the incoming data
char *inParse[64];                                         // Buffer for the parsed data chunks


String inString = "";                                      // Storage for data as string
int chindex = 0;
boolean stringComplete = false;


// ================================== Variables related to GCode Processing =====================================================
        

#define MAX_BUF (64) // What is the longest message Arduino can store?
#define STEPS_PER_TURN (400) // depends on your stepper motor. most are 200.
#define MAX_FEEDRATE (10000)
#define MIN_FEEDRATE (1)

char buffer[MAX_BUF]; // where we store the message until we get a ';'
int sofar; // how much is in the buffer
float px, py; // location
// speeds
float fr=0; // human version
// machine version
long step_delay_ms;
long step_delay_us;
// settings
char mode_abs=1; // absolute mode??
long line_number=0;

// ================================== End of defines and variables ==============================================================


void setup() {

// set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);         

  Serial.begin (115200);
  Serial.print("Linear Encoder Test");
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);

  pinMode(XaxisPWM, OUTPUT);
  analogWriteFrequency(XaxisPWM, 46875);     // Place PWM freq outside of audible range
  pinMode(XaxisDir, OUTPUT);
  
  pinMode(YaxisPWM, OUTPUT);
  analogWriteFrequency(YaxisPWM, 46875);     // Place PWM freq outside of audible range
  pinMode(YaxisDir, OUTPUT);
  

  if (timeStatus()!= timeSet) {
  Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  
  lcd.begin(20, 4);                             // set up the LCD's number of rows and columns: 

  lcd.print("Linear Encoder Test");
  lcd.setCursor(0, 1);
  lcd.print("02-15-2015");
  serialTimer.begin(timerInt,GUI_UPDATE_TIME);	// run timerInt routine every X period
  stateTimer.begin(stateInt,STATE_UPDATE_TIME);	// run state machine update regularly

  xPosn.setup(); yPosn.setup();	                // Initialize Quad Decode counters
  xPosn.start(); yPosn.start();                 // Start Quad Decode position count	                       
  
  delay(1000);                                  // 1 second delay to show dislay
  randomSeed(analogRead(0));                    // Used to select random setpoints for testing
 
 
  help(); // say hello
  position(0,0); // set staring position
  feedrate(200); // set default speed
  ready();

 
  XaxisPID.SetMode(AUTOMATIC);                  // Turn on the PID loop 
  XaxisPID.SetSampleTime(sampleRate);           // Sets the sample rate 
  XaxisPID.SetOutputLimits(0-Spd,Spd);           // Set max speed for DC motors

  YaxisPID.SetMode(AUTOMATIC);                  // Turn on the PID loop 
  YaxisPID.SetSampleTime(sampleRate);           // Sets the sample rate 
  YaxisPID.SetOutputLimits(0-Spd,Spd);           // Set max speed for DC motors


}

void loop() {          // NOTE:  ONLY WORKING X-AXIS for this sketch
  
    rtX=xPosn.calcPosn();     // Get current Xaxis position
    XaxisPos=rtX; 	            // Realtime values of X,Y
    
    XaxisPID.Compute(); 
    analogWrite(XaxisPWM,abs(XaxisSpd));              // Apply PID speed to motor
    if(XaxisSpd < 0) {  // Determine direction of travel
      digitalWrite(XaxisDir,BACKWARD);  
    } else {
      digitalWrite(XaxisDir,FORWARD);
    }      
  
    rtY=yPosn.calcPosn();     // Get current Xaxis position
    YaxisPos=rtY; 	            // Realtime values of X,Y
   
    YaxisPID.Compute(); 
    analogWrite(YaxisPWM,abs(YaxisSpd));              // Apply PID speed to motor
    if(YaxisSpd < 0) {  // Determine direction of travel
      digitalWrite(YaxisDir,BACKWARD);  
    } else {
      digitalWrite(YaxisDir,FORWARD);
    }      
   
 
    if (doOutput) updateDisplay(); 

   if (doState) updateState(); 


    SerialEvent2();                                            // Grab characters from Serial
  
  // =======================   if serial data available, process it ========================================================================
  if (stringComplete)                   // if there's any serial available, read it:
  {
    ParseSerialData();                  // Parse the recieved data
    inString = "";                      // Reset inString to empty   
    stringComplete = false;             // Reset the system for further input of data
  }  
 

}


void timerInt(void){                                // Do this action every Interval timeout.
    doOutput=true;
}


void stateInt(void){                                // Do this action every Interval timeout.
    doState=true;
}


