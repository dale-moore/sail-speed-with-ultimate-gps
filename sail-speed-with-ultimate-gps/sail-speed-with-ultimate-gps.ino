
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "EEPROM.h"

const int GPSRX = 2;
const int GPSTX = 3;

// left most display
const int DATAPIN1 = 4;
const int LATCHPIN1 = 5;
const int CLOCKPIN1 = 6;

// middle display
const int DATAPIN2 = 7;
const int LATCHPIN2 = 8;
const int CLOCKPIN2 = 9;

// right most display
const int DATAPIN3 = 10;
const int LATCHPIN3 = 11;
const int CLOCKPIN3 = 12;

const int PUSHBUTTONPIN = 13;

float speed = 00.0;
float displayedSpeed = 0.0;

int firstNumberDisplayed = 99;
int secondNumberDisplayed = 99;
int thirdNumberDisplayed = 99;

int firstNumber = 0;
int secondNumber = 0;
int thirdNumber = 0;

int haveFix = 0;
int segmentToLight = 0;  // this is done when we do not have a fix

// Handle push button
int keyPressedStartMillis = 0;
int totalKeyPressedMillis = 0;
int keyNotPressedStartMillis = 0;
int totalKeyNotPressedMillis = 0;

int mode = 0;  // 0=show speed, 1=set units
// 0 = mph, 1 = kph, 2 = knots
int units = 0;  
// this is saved to EEPROM at startup and then only updated if the user changes it. 
int unitsSaved = 0;

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
///SoftwareSerial mySerial(3, 2);
SoftwareSerial mySerial(GPSRX, GPSTX);  // RX,TX


// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):
//HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


/////////////////////////////////////////////////////////////////////////
// Arduino Setup
/////////////////////////////////////////////////////////////////////////
void setup()  
{
    // Enable the pins for output
    pinMode(DATAPIN1, OUTPUT);
    pinMode(LATCHPIN1, OUTPUT);
    pinMode(CLOCKPIN1, OUTPUT);
    
    pinMode(DATAPIN2, OUTPUT);
    pinMode(LATCHPIN2, OUTPUT);
    pinMode(CLOCKPIN2, OUTPUT);
    
    pinMode(DATAPIN3, OUTPUT);
    pinMode(LATCHPIN3, OUTPUT);  
    pinMode(CLOCKPIN3, OUTPUT);
   
    pinMode(PUSHBUTTONPIN, INPUT); 
    
    // Read the units from EEPROM.
    // For now, just default to KNOTS.
    if (EEPROM.read(1) == 255) {
      // default to knots
      EEPROM.write(0, 2);
      // write a byte that tells us we have been initialized
      EEPROM.write(1, 1);
    }
    
    unitsSaved = EEPROM.read(0);
    units = unitsSaved;
    
         
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Serial.begin(115200);
    Serial.println("Small Sailboat Speedometer Log!");
    
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    
    // Set the update rate    
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);  // once every 5 seconds
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
    
    // Request updates on antenna status, comment out to keep quiet
    //GPS.sendCommand(PGCMD_ANTENNA);
    
    // the nice thing about this code is you can have a timer0 interrupt go off
    // every 1 millisecond, and read data from the GPS for you. that makes the
    // loop code a heck of a lot easier!
    useInterrupt(true);
    
    delay(500);
    
    // Ask for firmware version
    mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    #ifdef UDR0
    if (GPSECHO)
        if (c) UDR0 = c;  
        // writing direct to UDR0 is much much faster than Serial.print 
        // but only one character can be written at a time. 
    #endif
}

/////////////////////////////////////////////////////////////////////////
// Interupt for GPS
/////////////////////////////////////////////////////////////////////////
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


/////////////////////////////////////////////////////////////////////////
// Arduino Loop
/////////////////////////////////////////////////////////////////////////
void loop() {                 
      
    if (mode == 0)
    {
        // Show the speed
        showSpeed();
    }
    else if (mode == 1)
    {
        // Show the units
        showUnits();
    }
    
    // Process the momentary switch
    // Check if the pushbutton has been pressed.
    if (digitalRead(PUSHBUTTONPIN) == LOW) 
    {
        // The key is not pressed
        keyPressedStartMillis = 0;
        totalKeyPressedMillis = 0;
        if (keyNotPressedStartMillis == 0)
        {
            keyNotPressedStartMillis = millis();
        }
        totalKeyNotPressedMillis = millis() - keyNotPressedStartMillis;
    }
    else 
    {
        // The key is pressed
        keyNotPressedStartMillis = 0;
        totalKeyNotPressedMillis = 0;
        if (keyPressedStartMillis == 0) 
        {
            keyPressedStartMillis = millis();
        }
        totalKeyPressedMillis = millis() - keyPressedStartMillis;
    }
    
    if (mode == 0)
    {
        // Display mode.
        // Watch for the user pressing the key for 5 seconds
        if (totalKeyPressedMillis > 3000)
        {
            Serial.println("Switching to unit mode");
            totalKeyPressedMillis = 0;
            keyPressedStartMillis = 0;
            mode = 1;
            
        }
    }
    else if (mode == 1)
    {
        firstNumberDisplayed = 99;
        secondNumberDisplayed = 99;
        thirdNumberDisplayed = 99;
        
        // Units mode
        // While in units mode, if a key is not pressed for 5 seconds
        // then switch back to display mode.
        if (totalKeyNotPressedMillis > 5000)
        {
            Serial.println("Switching to display mode due to 5 second timeout");
            totalKeyNotPressedMillis = 0;
            keyNotPressedStartMillis = 0;
            mode = 0;

            // If the units have changed save them
            if (unitsSaved != units)
            {
                EEPROM.write(0, units);
                unitsSaved = units;
            }            
        }
        else if (totalKeyPressedMillis > 500)
        {
            Serial.println("Changing units");
            totalKeyPressedMillis = 0;
            keyPressedStartMillis = 0;
            units = units + 1;
            if (units > 2) 
            {
                units = 0;
            }
        } 
    }
}

/////////////////////////////////////////////////////////////////////////
// showSpeed
/////////////////////////////////////////////////////////////////////////
void showSpeed()
{
    // if a sentence is received, we can check the checksum, parse it...        
    if (GPS.newNMEAreceived()) 
    {      
        Serial.println("GPS NMEA Message Received");    
        
        GPS.parse(GPS.lastNMEA());  
           
        if (GPS.fix) 
        {                      
            haveFix = 1;
            
            // Save the speed in knots
            speed = GPS.speed;
            
            Serial.print("GPS FIX MADE - GPS LAT: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", GPS LON:  "); 
            Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);            
            Serial.print(", "); 
            Serial.print("Fix Quality: " );  Serial.print(GPS.fixquality);                        
            Serial.print(", ");
            Serial.print("Speed (knots): "); Serial.print(GPS.speed);
            Serial.print(", ");
            Serial.print("Satellites: "); Serial.print((int)GPS.satellites);
            Serial.print(", ");
            Serial.print("Angle: "); Serial.print(GPS.angle);
            Serial.print(", ");
            Serial.print("Altitude: "); Serial.println(GPS.altitude);            
               
        }
        else 
        {
            haveFix = 0;
            
            // Since we don't have a fix set the speed to 0.
            speed = 0.0;
            
            Serial.println("No GPS Fix");
        }  
    }
    
    
    
    if (haveFix == 1)
    {
        // If the speed is less than 1 knot then ignore it
        if (speed < 0.5)
        {
            speed = 0.0;
        }
        
        // speed is knots from the GPS
        if (units == 0) 
        {
            // Convert speed to mph
            displayedSpeed = speed * 1.15077945;        
            Serial.println("Showing mph");
        }
        else if (units == 1)
        {
            // Convert the speed to kilometers
            displayedSpeed = speed * 1.85200;
            Serial.println("Showing kilometers");
        }
        else 
        {
            // Convert the speed to knots
            displayedSpeed = speed;           
            Serial.println("Showing knots"); 
        }
        
        
        
        // Convert speed to kph
        
        
        calculateNumber(displayedSpeed);

        int numberUpdated = 0;
        if (firstNumberDisplayed != firstNumber)
        {
          // Update the number
          numberUpdated = 1;
          showNumberViaShiftRegisters(firstNumber, DATAPIN1, LATCHPIN1, CLOCKPIN1);
          firstNumberDisplayed = firstNumber;
        }
        if (secondNumberDisplayed != secondNumber)
        {
          // Update the number
          numberUpdated = 1;
          showNumberViaShiftRegisters(secondNumber, DATAPIN2, LATCHPIN2, CLOCKPIN2);
          secondNumberDisplayed = secondNumber;
        }
        if (thirdNumberDisplayed != thirdNumber)
        {
          // Update the number
          numberUpdated = 1;
          showNumberViaShiftRegisters(thirdNumber, DATAPIN3, LATCHPIN3, CLOCKPIN3);
          thirdNumberDisplayed = thirdNumber;
        }
        
        if (numberUpdated == 1)
        {
            Serial.print("The speed is ");
            Serial.print(speed);
            Serial.print(".  ");
            Serial.print("first number=");
            Serial.print(firstNumber);
            Serial.print(" second number=");
            Serial.print(secondNumber);
            Serial.print(" third number=");
            Serial.println(thirdNumber);            
        }
                      
        delay(100);
    }
    else
    {
        // Do not have a fix so show animation
        lightSegmentViaShiftRegisters(segmentToLight, DATAPIN1, LATCHPIN1, CLOCKPIN1);
        lightSegmentViaShiftRegisters(segmentToLight, DATAPIN2, LATCHPIN2, CLOCKPIN2);
        lightSegmentViaShiftRegisters(segmentToLight, DATAPIN3, LATCHPIN3, CLOCKPIN3);
        
        // increment the segment
        segmentToLight = segmentToLight + 1;
        
        if (segmentToLight > 5) {
            segmentToLight = 0;
        }
        delay(100);
    }
}

/////////////////////////////////////////////////////////////////////////
// showUnits
/////////////////////////////////////////////////////////////////////////
void showUnits() 
{
    
    if (units == 0)
    {
        // Show MPH. 
        // 1st segment - show sideways M using pins A, F, G, E, D
        // 2nd segment - shows P using pins A, B, G, F, E
        // 3rd segment - shows H using pins F, G, E, C
        showLetterViaShiftRegisters('m', DATAPIN1, LATCHPIN1, CLOCKPIN1);
        showLetterViaShiftRegisters('p', DATAPIN2, LATCHPIN2, CLOCKPIN2);
        showLetterViaShiftRegisters('h', DATAPIN3, LATCHPIN3, CLOCKPIN3);
    }
    else if (units == 1)
    {
        // Show KIL
        // Show KPH  |- P h
        // 1st segment - shows k using pins F, E, G, D
        // 2nd segment - shows P using pins A, B, G, F, E
        // 3rd segment - shows H using pins F, G, E, C
        showLetterViaShiftRegisters('k', DATAPIN1, LATCHPIN1, CLOCKPIN1);
        showLetterViaShiftRegisters('p', DATAPIN2, LATCHPIN2, CLOCKPIN2);
        showLetterViaShiftRegisters('h', DATAPIN3, LATCHPIN3, CLOCKPIN3);
    }
    else if (units == 2)
    {
        // Show KNOTS - nph
        // 1st segment - show n using pins E, G, C
        // 2nd segment - show o using pins G, E, D, C
        // 3rd segment - show t using pins A, B, C 
        showLetterViaShiftRegisters('n', DATAPIN1, LATCHPIN1, CLOCKPIN1);
        showLetterViaShiftRegisters('o', DATAPIN2, LATCHPIN2, CLOCKPIN2);
        showLetterViaShiftRegisters('t', DATAPIN3, LATCHPIN3, CLOCKPIN3);
    }
        
}


/////////////////////////////////////////////////////////////////////////
// calculateNumber
// this function takes the speed and determines what numbers each of the
// 7 segment LEDs should show.
/////////////////////////////////////////////////////////////////////////
void calculateNumber(float aNumber)
{
  // Determine first digit
  int wholeNumber = aNumber * 10;

  // From left to right
  firstNumber = wholeNumber / 100;
  secondNumber = (wholeNumber - (firstNumber * 100)) / 10;
  thirdNumber = wholeNumber - (firstNumber * 100) - (secondNumber * 10);
}

/////////////////////////////////////////////////////////////////////////
// lightSegmentViaShiftRegisters
/////////////////////////////////////////////////////////////////////////
void lightSegmentViaShiftRegisters(int aSegment, int aDataPin, int aLatchPin, int aClockPin)
{
    // GFE DCBA    
    // 0000,0001 will be written as 1 for Q0
    // 0000,0010 will be written as 1 for Q1
    //
    // 0000,0001 - 1
    // 0000,0010 - 2
    // 0000,0100 - 4
    // 0000,1000 - 8
    // 0001,0000 - 16
    // 0010,0000 - 32
    // 0100,0000 - 64

    int shiftNumber = 0;
    if (aSegment == 0) {
        shiftNumber = 1;
    }
    else if (aSegment == 1) {
        shiftNumber = 2;
    }
    else if (aSegment == 2) {
        shiftNumber = 4;
    }
    else if (aSegment == 3) {
        shiftNumber = 8;
    }
    else if (aSegment == 4) {
        shiftNumber = 16;
    }
    else if (aSegment == 5) {
        shiftNumber = 32;
    }
    else if (aSegment == 6) {
        shiftNumber = 64;
    }
          
    digitalWrite(aLatchPin, LOW);
    shiftOut(aDataPin, aClockPin, MSBFIRST, shiftNumber);    
    digitalWrite(aLatchPin, HIGH);
}

/////////////////////////////////////////////////////////////////////////
// showNumberViaShiftRegisters
// h, k, m, n, o, p, t
/////////////////////////////////////////////////////////////////////////
void showLetterViaShiftRegisters(char aLetter, int aDataPin, int aLatchPin, int aClockPin) 
{
    // pin definitions
    int a = 1;
    int b = 2;
    int c = 4;
    int d = 8;
    int e = 16;
    int f = 32;
    int g = 64;
    
    //  GFE DCBA    
    // 0000,0001 will be written as 1 for Q0
    // 0000,0010 will be written as 1 for Q1
    int letter = 0;
    if (aLetter == 'h') 
    {
        letter = f | e | g | b | c;
    }
    else if (aLetter == 'k')
    {
        letter = f | e | g;
    }
    else if (aLetter == 'm')
    {
        letter = a | f | g | e | d;
    }
    else if (aLetter == 'n') 
    {
        letter = e | g | c;
    }
    else if (aLetter == 'o')
    {
        letter = e | g | c | d;
    }    
    else if (aLetter == 'p')
    {
        letter = e | f | a | b | g;
    }
    else if (aLetter == 't')
    {
        letter = c | g;
    }
    else 
    {
        letter = a | b | c | d | e | f | g;
    }
    
    digitalWrite(aLatchPin, LOW);
    shiftOut(aDataPin, aClockPin, MSBFIRST, letter);    
    digitalWrite(aLatchPin, HIGH);

}

/////////////////////////////////////////////////////////////////////////
// showNumberViaShiftRegisters
// numbers 0-9 are understood
/////////////////////////////////////////////////////////////////////////
void showNumberViaShiftRegisters(int aNumber, int aDataPin, int aLatchPin, int aClockPin)
{
    //  GFE DCBA    
    // 0000,0001 will be written as 1 for Q0
    // 0000,0010 will be written as 1 for Q1
    //
    // 0000,0110 is 1 is 6 ok
    // 0101,1011 is 2 is 91
    // 0100,1111 is 3 is 79
    // 0110,0110 is 4 is 102
    // 0110,1101 is 5 is 109
    // 0111,1101 is 6 is 125
    // 0000,0111 is 7 is 7
    // 0111,1111 is 8 is 127
    // 0110,1111 is 9 is 111
    // 0011,1111 is 0 is 63
    // 
    
    int shiftNumber = 0;
    if (aNumber == 0)
      shiftNumber = 63;
    else if (aNumber == 1)
      shiftNumber = 6;
    else if (aNumber == 2)
      shiftNumber = 91;
    else if (aNumber == 3)
      shiftNumber = 79;
    else if (aNumber == 4)
      shiftNumber = 102;
    else if (aNumber == 5)
      shiftNumber = 109;
    else if (aNumber == 6)
      shiftNumber = 125;
    else if (aNumber == 7)
      shiftNumber = 7;
    else if (aNumber == 8)
      shiftNumber = 127; 
    else if (aNumber == 9)
      shiftNumber = 111;
      
    digitalWrite(aLatchPin, LOW);
    shiftOut(aDataPin, aClockPin, MSBFIRST, shiftNumber);    
    digitalWrite(aLatchPin, HIGH);
}

