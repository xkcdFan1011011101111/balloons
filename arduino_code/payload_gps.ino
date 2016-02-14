/*
 * Arduino code for the GPS only Payload
 * 
 * 
 * connections:
 * GPS TX <-> Arduino GPIO 3
 * GPS RX <-> Arduino GPIO 2
 * openLog RX <-> Arduino GPIO 1
 * openLog TX <-> Arduino GPIO 0
 * activity LED anode : Arduino GPIO 12
 * 
 * 
 * Sept 28, 2015
 * Joseph Galante
 */

// set up GPS

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define gpsRxPin 3 // connect pin Digital3 to the GPS unit's transmit pin
#define gpsTxPin 2 // connect pin Digital2 to the GPS unit's receive pin
SoftwareSerial mySerial = SoftwareSerial(gpsRxPin, gpsTxPin);//constructor(rxPin,txPin)
Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
//#define GPSECHO  true
#define GPSECHO false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


// set up blue activity LED
#define activityLEDpin 12 // connect pin Digital12 to the resistor and blue LED

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  // set up activity LED
  pinMode(activityLEDpin,OUTPUT);
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

uint32_t timer = millis();
void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer

    
    if (GPS.fix){
      // if we have a GPS fix, write out the GPS data
      // print Arduino clock's time
      Serial.print(millis(),DEC);         Serial.print(',');
      // print GPS year, month, day, hour, minute, second, millisecond
      Serial.print(GPS.year,DEC);         Serial.print(',');
      Serial.print(GPS.month,DEC);        Serial.print(',');
      Serial.print(GPS.day,DEC);          Serial.print(',');
      Serial.print(GPS.hour,DEC);         Serial.print(',');
      Serial.print(GPS.minute,DEC);       Serial.print(',');
      Serial.print(GPS.seconds,DEC);      Serial.print(',');
      Serial.print(GPS.milliseconds,DEC); Serial.print(',');
      // print fix, quality, number satellites
      Serial.print((int)GPS.fix);         Serial.print(',');
      Serial.print((int)GPS.fixquality);  Serial.print(',');
      Serial.print((int)GPS.satellites);  Serial.print(',');
      // lattitude
      Serial.print(GPS.latitudeDegrees, 4);     Serial.print(',');
      // longitude
      Serial.print(GPS.longitudeDegrees, 4);    Serial.print(',');
      // altitude
      Serial.print(GPS.altitude);               Serial.print(',');
      // speed
      Serial.print(GPS.speed);                  Serial.print(',');
      // heading
      Serial.print(GPS.angle);                  Serial.print(',');           
      // new line
      Serial.print('\n');
      // light activity LED to indicate we are getting GPS
      digitalWrite(activityLEDpin,HIGH);
    }else{
      // if we don't have a GPS fix, write out a bunch of zeros
      // print Arduino clock's time
      Serial.print(millis(),DEC);         Serial.print(',');
      // print year, month, day, hour, minute, second, millisecond
      Serial.print(GPS.year,DEC);         Serial.print(',');
      Serial.print(GPS.month,DEC);        Serial.print(',');
      Serial.print(GPS.day,DEC);          Serial.print(',');
      Serial.print(GPS.hour,DEC);         Serial.print(',');
      Serial.print(GPS.minute,DEC);       Serial.print(',');
      Serial.print(GPS.seconds,DEC);      Serial.print(',');
      Serial.print(GPS.milliseconds,DEC); Serial.print(',');
      // print fix, quality, number satellites
      Serial.print("0,");
      Serial.print("0,");
      Serial.print("0,");
      // lattitude
      Serial.print("0,");
      // longitude
      Serial.print("0,");
      // altitude
      Serial.print("0,");
      // speed
      Serial.print("0,");
      // heading
      Serial.print("0,");    
             
      // new line
      Serial.print('\n');
      // turn off activity LED to indicate we aren't getting GPS
      digitalWrite(activityLEDpin,LOW);
    }
   
  }
}
