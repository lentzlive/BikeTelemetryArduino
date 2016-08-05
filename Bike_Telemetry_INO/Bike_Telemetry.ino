#include <SoftwareSerial.h>
#include <Wire.h>
//
#include <TinyGPS++.h>
//
#include <ADXL345.h>
//
#include <Adafruit_Sensor.h>
#include "Adafruit_TMP006.h"
//
#include <NewPing.h>

/****************************************
    Variables for GPS Functions
 ***************************************/
const float alpha = 0.5;

double fXg = 0;
double fYg = 0;
double fZg = 0;

double refXg = 0;
double refYg = 0;
double refZg = 0;
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 12, TXPin = 13;
static const int GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
double Start_LAT = 0.0; // Per calcolo Distanza percorsa
double Start_LNG = 0.0; // Per calcolo Distanza percorsa
bool Start_CalculateDistance = false; // Per calcolo Distanza percorsa

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


/****************************************
    Variables for ADXL345 Functions
 ***************************************/
int i = 0;
ADXL345 acc;

/****************************************
    Variables for HC RS04 Functions
 ***************************************/
//HC RS04 - 1 (front)
int triggerPort = 2;
int echoPort = 3;

//HC RS04 - 2 (Back)
int triggerPort2 = 4;
int echoPort2 = 5;

NewPing sonar(triggerPort, echoPort, 200 );     // NewPing setup of pins and maximum distance.
NewPing sonar2(triggerPort2, echoPort2, 200 );  // NewPing setup of pins and maximum distance.



/****************************************
    Variables for Bluetooth Functions
 ***************************************/
SoftwareSerial BTSerial(10, 11); // RX | TX



/****************************************
    Variables for Audio Sensor Functions - Optional
 ***************************************/
int sensorPin = A0;       // select the input pin for the potentiometer
int ledPin = 13;          // select the pin for the LED
double sensorValue = 0;   // variable to store the value coming from the sensor

/****************************************
    Variables for External Temperature (LM35) - Optional
 ***************************************/
int TempExtPin = A1;                // select the input pin for the ExternalTemperature LM35
float celsius = 0, farhenheit = 0;  // temperature variables
float millivolts;                   //dichiarazione di variabile tensione (float è per i numeri con la virgola)
int sensor;

/****************************************
    Variables for TMP006  Functions
 ***************************************/
Adafruit_TMP006 tmp006;


void setup()
{
  pinMode(9, OUTPUT);                   // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);
  Serial.begin(9600);

  // Bluetooth Inizialize
  BTSerial.begin(9600);                 // HC-05 default speed in AT command more
  Serial.println("Setup HC-05 End");

  delay(3000);

  // GPS Inizialize
  Serial.println("Setup GPS START ...");
  ss.begin(GPSBaud);
  delay(3000);
  Serial.println("Setup GPS End");


  // ADXL345 Inizialize
  acc.begin();
  delay(1000);
  Serial.println("Setup ACC End");
  // Sensore audio
  pinMode(sensorPin, INPUT);

  i = 0;

  // HC RS04 Inizialize
  // HC RS04 - 1
  Serial.println("HCSR04_1 - Setup");
  pinMode( triggerPort, OUTPUT );
  pinMode( echoPort, INPUT );
  // HC RS04 - 2
  Serial.println("HCSR04_2 - Setup");
  pinMode( triggerPort2, OUTPUT );
  pinMode( echoPort2, INPUT );


  Serial.print(F("MyTelemetry Project V 1.03 "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Davide Longo - LentzLive"));
  Serial.println();

}

void loop()
{

  while (ss.available() > 0)
    if (gps.encode(ss.read()))
    {
      //displayInfo();
      if (gps.location.isValid())
        displayInfo();
      else
      {
        delay(5000);
        Serial.println("wait position");
      }
    }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }

}


void displayInfo()
{
  double distanceKm = 0.0;
  double distanceMeters = 0.0;

  /* ARRAY DEFINITION:

     0  - START
     1  - Latitude
     2  - N (Nord)
     3  - Longitude
     4  - E (East)
     5  - month
     6  - day
     7  - year
     8  - hh:MM.ss
     9  - speed (Km/h)
     10  - altitude (m)
     11 - satellites (number of satellites)
     12 - hdop (number of satellites in use)
     13 - roll
     14 - pitch
     15 - Xg
     16 - Yg
     17 - Zg
     18 - Audio (Optional-currently disable)
     19 - Distance (in metri)
     20 - Temperature (Esterna tramite LM35)
     21 - Temperature Tyre (Temperatura gomma posteriore tramite TMP006)
     22 - Ammortizzatore Anteriore
     23 - Ammortizzatore Posteriore
  */

  String strMessage = "";

  if (gps.location.isValid())
  {
    strMessage = "$START|";                       // 0

    double latitude = gps.location.lat();
    double longitude = gps.location.lng();

    strMessage += String(latitude, 6);            // 1
    strMessage += "|N|";                          // 2
    strMessage += String(longitude, 6);           // 3
    strMessage += "|E|";                          // 4

    /*   const double EIFFEL_TOWER_LAT = 40.909439;
       const double EIFFEL_TOWER_LNG = 9.520036;
       distanceMeters =
         gps.distanceBetween(
         gps.location.lat(),
         gps.location.lng(),
         EIFFEL_TOWER_LAT,
         EIFFEL_TOWER_LNG) / 1.0;
    */

    if (Start_LAT == 0.000000  && Start_LNG == 0.000000)
    {
      Start_CalculateDistance = true;
      Start_LAT = latitude;
      Start_LNG = longitude;
    }
    if (Start_CalculateDistance)
    {
      distanceMeters = gps.distanceBetween(latitude, longitude, Start_LAT, Start_LNG) / 1000;  // For compute KM divide per 1000
      Start_LAT = latitude;
      Start_LNG = longitude;
    }
  }
  else
  {
    strMessage = "$START|";                       // 0
    strMessage += "INVALID";                      // 1
    strMessage += "|N|";                          // 2
    strMessage += "INVALID";                      // 3
    strMessage += "|E|";                          // 4
  }

  /*
    Serial.print("Sentences that failed checksum=");
    Serial.println(gps.failedChecksum());

    // Testing overflow in SoftwareSerial is sometimes useful too.
    Serial.print("Soft Serial device overflowed? ");
    Serial.println(ss.overflow() ? "YES!" : "No");
    Serial.print("charsProcessed ");
    Serial.println(gps.charsProcessed());
    Serial.print("sentencesWithFix ");
    Serial.println(gps.sentencesWithFix());
    Serial.print("passedChecksum ");
    Serial.println(gps.passedChecksum());
  */


  // Data
  if (gps.date.isValid())
  {
    strMessage += gps.date.month();               // 5
    strMessage += "|";
    strMessage += gps.date.day();                 // 6
    strMessage += "|";
    strMessage += gps.date.year();                // 7
    strMessage += "|";


    String h = "";
    String m = "";
    String s = "";
    int iH = gps.time.hour();
    int iM = gps.time.minute();
    int iS = gps.time.second();

    if (iH < 10)
    {
      h = "0" + String(iH);
    }
    else
      h = String(iH);
    if (iM < 10)
    {
      m = "0" + String(iM);
    }
    else
      m = String(iM);
    if (iS < 10)
    {
      s = "0" + String(iS);
    }
    else
      s = String(iS);

    strMessage += h + ":" + m + ":" + s;            // 8
    strMessage += "|";
  }
  else
  {
    strMessage += "INVALID";                        // 5
    strMessage += "|";
    strMessage += "INVALID";                        // 6
    strMessage += "|";
    strMessage += "INVALID";                        // 7
    strMessage += "|";
    strMessage += "INVALID";                        // 8
    strMessage += "|";
  }

  // Speed
  strMessage += gps.speed.kmph();                   // 9
  strMessage += "|";
  // Altitude
  double alt = gps.altitude.meters();               // 10
  strMessage += String(alt, 6);
  strMessage += "|";

  // Number of satellites in use (u32)
  strMessage += String(gps.satellites.value());     // 11
  strMessage += "|";
  // Number of satellites in use (u32)
  strMessage += String(gps.hdop.value());           // 12
  strMessage += "|";

  // ADXL sensor
  double pitch, roll, Xg, Yg, Zg;
  acc.read(&Xg, &Yg, &Zg);

  // Calibration ADXL345
  if (i == 0)
  {
    refXg = Xg; refYg = Yg; refZg = Zg;
    i = 1;
  }

  Xg = Xg - refXg;
  Yg = Yg - refYg;
  Zg = Zg - refZg + 1;

  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));

  // Roll & Pitch Equations
  roll  = -(atan2(-fYg, fZg) * 180.0) / M_PI; 
  pitch = (atan2(fXg, sqrt(fYg * fYg + fZg * fZg)) * 180.0) / M_PI;

  strMessage += roll;                     // 13
  strMessage += "|";
  strMessage += pitch;                    // 14
  strMessage += "|";
  strMessage += Xg;                       // 15
  strMessage += "|";
  strMessage += Yg;                       // 16
  strMessage += "|";
  strMessage += Zg;                       // 17
  strMessage += "|";


  // Sensore audio
  //sensorValue = analogRead(sensorPin);    // 18
  strMessage += "-";//String(sensorValue, 4);
  strMessage += "|";

  strMessage += String(distanceMeters, 2);     // 19
  strMessage += "|";

  float objt = tmp006.readObjTempC();
  //float diet = tmp006.readDieTempC();

  // External Temperature LM35
    sensor = analogRead(TempExtPin);          //lettura valore del sensore LM35 messo sull'ingresso analogico A1
    millivolts = ( sensor / 1023.0) * 5000;   //formula per ottenere la tensione di uscita dell'LM35 in millivolts
    celsius = millivolts / 10;                // valore espresso in gradi Celsius (l'out del sensore è 10mv per grado)

  strMessage += celsius;  // String(diet, 2);            // 20
  strMessage += "|";

  strMessage += String(objt, 2);            // 21
  strMessage += "|";


  /**** POSITIONS *****/

  int Pos1 = ReadPositionPing();
  if (Pos1 < 9999.9)
  {
    strMessage += Pos1;          // 22
    strMessage += "|";
  }
  else
  {
    strMessage += "-";                      // 22
    strMessage += "|";
  }

  int Pos2 = ReadPosition2Ping();
  if (Pos2 < 9999)
  {
    strMessage += Pos2;          // 23
    strMessage += "|";
  }
  else
  {
    strMessage += "-";                      // 23
    strMessage += "|";
  }


  strMessage += "$END|";
  
  // comment the line below before deploy
  Serial.println(strMessage);  
  
  //if (ss.overflow() )
  //  delay(10);


  BTSerial.print(strMessage);
}


/*********************************************/
/***********      FUNCTIONS  ****************/
/********************************************/


// Positions HCSR04
int ReadPositionPing() {
  unsigned int uS = sonar.ping();
  return sonar.convert_cm(uS);
}

int ReadPosition2Ping() {
  unsigned int uS = sonar2.ping();
  return sonar.convert_cm(uS);
}




