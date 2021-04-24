//*************DEBUG****************

//#define DEBUG                                     //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG                                      //Macros are usually in all capital letters.
#define DPRINT(...) Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DPRINTLN(...) Serial.println(__VA_ARGS__) //DPRINTLN is a macro, debug print with new line
#warning
#else
#define DPRINT(...)   //now defines a blank line
#define DPRINTLN(...) //now defines a blank line
#endif

//*******Welcher Arduino?
#define Arduino_1
//#define Arduino_2

#if (defined(Arduino_1) == defined(Arduino_2))
#error
#endif

//***********Bibliotheken***************

//SD-Datalogger
#include <SPI.h>
#include <SD.h>

//BMP180
#ifdef Arduino_1
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;
#endif

//BMP390
#ifdef Arduino_2
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
Adafruit_BMP3XX bmp;
#endif

//GPS
#ifdef Arduino_1
#include <SoftwareSerial.h> // serial connection to the GPS device
const int _PIN_GPS_RX = 6;
const int _PIN_GPS_TX = 8;
SoftwareSerial gpsSerial(_PIN_GPS_TX, _PIN_GPS_RX);
#endif

//Piper
#ifdef Arduino_1
#include <Servo.h>
Servo piper; // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;
#endif

//MPU6050
#ifdef Arduino_2
//#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
#endif

//****PinBelegung

const int _PIN_BMP_SDA = 2;
const int _PIN_BMP_SCL = 3;
const int _PIN_TMP36 = 4;
const int _PIN_DVR_TRIGGER = 5;
//const int _PIN_GPS_RX = 6; //unter setup GPS
//const int _PIN_GPS_TX = 8;
const int _PIN_CAM_EA = 7;
const int _PIN_Piper = 9;
const int _PIN_SD_CS = 10;
const int _PIN_ServicePort = 18;
const int _PIN_LED_ROT = 19;
const int _PIN_LED_GELB = 20;
const int _PIN_LED_GRUEN = 21;

int ledState = LOW;

/*
   SD card SPI bus: (intern festgelegt)
   SCLK - pin 15
   MISO - pin 14
   MOSI - pin 16
   CS - siehe oben

*/
//Luftdruck am Boden
#ifdef Arduino_1
float ground_pressure = 101325; //auf Standartwert gesetzt -> später reset auf 0m
#endif

#ifdef Arduino_2
float ground_pressure = 1013.25; //auf Standartwert gesetzt -> später reset auf 0m
#endif

//****Daten2Log
String dataString = "timestamp,system_state,bmp_pressure,bmp_alt,bmp_temp,tmp36_temp,cam_mosfet_ea,dvr_trigger,mpu_acX,mpu_acY,mpu_acZ,mpu_temp,mpu_gyX,mpu_gyY,mpu_gyZ,gps_sats,gps_VDOP,gps_HDOP,gps_PDOP,gps_Latitude,gps_Longitude,gps_FixAge,gps_Date,gps_Time,gps_DateAge,gps_altitude,gps_Course,gps_Speed,gps_Card";
unsigned long previousTime;

int system_state = 0;
float bmp_pressure = 0;
float bmp_alt = 0;
float bmp_temp = 0;
int tmp36_temp = 0;
bool cam_mosfet_ea = 0;
bool dvr_trigger = 1;
float mpu_acX = 0;
float mpu_acY = 0;
float mpu_acZ = 0;
float mpu_temp = 0;
float mpu_gyX = 0;
float mpu_gyY = 0;
float mpu_gyZ = 0;
float gps_sats = 0;
float gps_VDOP = 0;
float gps_HDOP = 0;
float gps_PDOP = 0;
float gps_Latitude = 0;
float gps_Longitude = 0;
float gps_FixAge = 0;
float gps_Date = 0;
float gps_Time = 0;
float gps_DateAge = 0;
float gps_altitude = 0;
float gps_Course = 0;
float gps_Speed = 0;
float gps_Card = 0; //????

void setup()
{
#ifdef DEBUG
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  pinMode(_PIN_CAM_EA, OUTPUT); //MOSFET der Kameras auf LOW

  setup_LEDs();  

  setup_SD();

#ifdef Arduino_1
  setup_BMP180();

  gpsSerial.begin(9600);

  piper.attach(9);
#endif

#ifdef Arduino_2
  setup_BMP390();
  setup_MPU();
#endif
}

void loop()
{
  // put your main code here, to run repeatedly:

  read_tmp36();

#ifdef Arduino_1
  read_BMP180();
  read_GPS();
  mute_piper();
#endif

#ifdef Arduino_2
  read_BMP390();
  read_MPU();
#endif
  update_system_state();
  data_to_dataString();
  dataString_to_SD();
  //delay(1000);
}

//*************Funktionen***********

void setup_LEDs(){
  pinMode(_PIN_LED_ROT, OUTPUT); //LEDs als Ausgang
  pinMode(_PIN_LED_GELB, OUTPUT);
  pinMode(_PIN_LED_GRUEN, OUTPUT);
  digitalWrite(_PIN_LED_ROT, HIGH);//alle LEDs kurz an
  digitalWrite(_PIN_LED_GELB, HIGH);
  digitalWrite(_PIN_LED_GRUEN, HIGH);
  delay(2000);
  digitalWrite(_PIN_LED_ROT, LOW);//alle LEDs aus
  digitalWrite(_PIN_LED_GELB, LOW);
  digitalWrite(_PIN_LED_GRUEN, LOW);

}

void setup_SD()
{
  DPRINT("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(_PIN_SD_CS))
  {
    DPRINTLN("Card failed, or not present");
    // don't do anything more:
    digitalWrite(_PIN_LED_ROT, HIGH);
    //while (1); //*********************************************************************Achtung Fehlermeldung
  }
  DPRINTLN("card initialized.");

  dataString_to_SD();
}

#ifdef Arduino_1
void setup_BMP180()
{
  if (!bmp.begin())
  {
    DPRINTLN("Could not find a valid BMP085 sensor, check wiring!");
    digitalWrite(_PIN_LED_ROT, HIGH);
    //while (1); //*********************************************************************Achtung Fehlermeldung
  }
  read_BMP180();
  ground_pressure = bmp_pressure;
}
#endif

#ifdef Arduino_2
void setup_BMP390()
{
  if (!bmp.begin_I2C())
  { // hardware I2C mode, can pass in address & alt Wire
    DPRINTLN("Could not find a valid BMP3 sensor, check wiring!");
    digitalWrite(_PIN_LED_ROT, HIGH);
    //while (1); //*********************************************************************Achtung Fehlermeldung
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  read_BMP390();
  ground_pressure = bmp_pressure / 100;
}
#endif

#ifdef Arduino_2
void setup_MPU()
{
  Wire.begin();                // Initialize comunication
  Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);            // Talk to the register 6B
  Wire.write(0x00);            // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x18); //Set the register bits as 00011000 (+/- 16g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x18); // Set the register bits as 00011000 (2000deg/s full scale)
  Wire.endTransmission(true);

  delay(100);
}
#endif

//LOOP Funktionen

void read_tmp36()
{
  tmp36_temp = analogRead(_PIN_TMP36);
}

#ifdef Arduino_1
void read_BMP180()
{
  bmp_pressure = bmp.readPressure();
  bmp_alt = bmp.readAltitude(ground_pressure);
  bmp_temp = bmp.readTemperature();
}
#endif

#ifdef Arduino_2
void read_BMP390()
{
  if (!bmp.performReading())
  {
    DPRINTLN("Failed to perform reading :(");
    return;
  }
  bmp_pressure = bmp.pressure;
  bmp_alt = bmp.readAltitude(ground_pressure);
  bmp_temp = bmp.temperature;
}
#endif

#ifdef Arduino_1
void read_GPS()
{
  // Output raw GPS data to the serial monitor
  if (gpsSerial.available() > 0)
  {
    Serial.println("GPS:");
    File dataFile = SD.open("datalog.gps", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
      while (gpsSerial.available() > 0)
      {
        //Serial.write(gpsSerial.read());
        dataFile.write(gpsSerial.read());
      }
      dataFile.close();
      // print to the serial port too:
      DPRINTLN("GPS geschrieben");
    }
    // if the file isn't open, pop up an error:
    else
    {
      DPRINTLN("error opening datalog.txt"); //*********************************************************************Achtung Fehlermeldung
    }
  }
}
#endif

#ifdef Arduino_1
void mute_piper()
{
  for (pos = 0; pos <= 180; pos += 50)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    piper.write(pos); // tell servo to go to position in variable 'pos'
    // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 50)
  {                   // goes from 180 degrees to 0 degrees
    piper.write(pos); // tell servo to go to position in variable 'pos'
    // waits 15ms for the servo to reach the position
  }
}
#endif

#ifdef Arduino_2
void read_MPU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Read 6 registers total, each axis value is stored in 2 registers

  mpu_acX = (Wire.read() << 8 | Wire.read()); // X-axis value
  mpu_acY = (Wire.read() << 8 | Wire.read()); // Y-axis value
  mpu_acZ = (Wire.read() << 8 | Wire.read()); // Z-axis value
  mpu_temp = (Wire.read() << 8 | Wire.read());
  mpu_gyX = (Wire.read() << 8 | Wire.read());
  mpu_gyY = (Wire.read() << 8 | Wire.read());
  mpu_gyZ = (Wire.read() << 8 | Wire.read());
}
#endif

void update_system_state()
{
  switch (system_state)
  {
  case 0:
    if (bmp_alt > 200)
    {
      digitalWrite(_PIN_CAM_EA, HIGH); //Kamera Mosfett ein
      cam_mosfet_ea = 1;
      previousTime = millis();         //Sytemzeit abspeichern
      system_state = 1;
      digitalWrite(_PIN_LED_GRUEN, LOW);
      digitalWrite(_PIN_LED_GELB, HIGH);
      break;
    }

    if (millis() - previousTime > 1000)
    {
      previousTime = millis();         //Sytemzeit abspeichern

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(_PIN_LED_GRUEN, ledState);
    }

    /* Alternative 2Byte Programmspeicher mehr, dynamischer Speicher 2Byte weniger
    if (millis() - previousTime > 1000)
    {
      digitalWrite(_PIN_LED_GRUEN, HIGH);
    }
    if (millis() - previousTime > 2000)
    {
      digitalWrite(_PIN_LED_GRUEN, LOW);
    }
     if (millis() - previousTime > 3000)
    {
      previousTime = millis();         //Sytemzeit abspeichern
    }
    */

    break;

  case 1:
    if (millis() - previousTime > 5000)
    {
      pinMode(_PIN_DVR_TRIGGER, OUTPUT);   //DVR Trigger LOW
      digitalWrite(_PIN_DVR_TRIGGER, LOW); //DVR Trigger LOW
      dvr_trigger = 0;
      previousTime = millis();             //Systemzeit abspeichern
      system_state = 2;
    }

    break;

  case 2:
    if (millis() - previousTime > 1000)
    {
      pinMode(_PIN_DVR_TRIGGER, INPUT); //DVR Trigger Input (loslassen)
      dvr_trigger = 1;
      previousTime = millis();          //Systemzeit abspeichern
      system_state = 3; 
      digitalWrite(_PIN_LED_GELB, HIGH); //gelbe LED an
    }

    break;

  case 3:
    if (millis() - previousTime > 30000)//*********************************************Zeit anpassen
    {
      pinMode(_PIN_DVR_TRIGGER, OUTPUT);   //DVR Trigger LOW
      digitalWrite(_PIN_DVR_TRIGGER, LOW); //DVR Trigger LOW
      dvr_trigger = 0;
      previousTime = millis();             //Systemzeit abspeichern
      system_state = 4;
    }

    break;

  case 4:
    if (millis() - previousTime > 1000)
    {
      pinMode(_PIN_DVR_TRIGGER, INPUT); //DVR Trigger Input (loslassen)
      dvr_trigger = 1;
      previousTime = millis();          //Systemzeit abspeichern
      system_state = 5;
    }

    break;

  case 5:
    if (millis() - previousTime > 8000)
    {
      digitalWrite(_PIN_CAM_EA, LOW); //Kamera Mosfett aus
      cam_mosfet_ea = 0;
      digitalWrite(_PIN_LED_GELB, LOW);
      previousTime = millis();        //Systemzeit abspeichern
      system_state = 6;
    }

    break;

  case 6:
    digitalWrite(_PIN_LED_GRUEN, HIGH); //grüne LED an
    #ifdef Arduino_1
    piper.detach();
    #endif
    break;
  }
}

void data_to_dataString()
{
  dataString = "";
  dataString += String(millis()) + ",";
  dataString += String(system_state) + ",";
  dataString += String(bmp_pressure) + ",";
  dataString += String(bmp_alt) + ",";
  dataString += String(bmp_temp) + ",";
  dataString += String(tmp36_temp) + ",";
  dataString += String(cam_mosfet_ea) + ",";
  dataString += String(dvr_trigger) + ",";
  dataString += String(mpu_acX) + ",";
  dataString += String(mpu_acY) + ",";
  dataString += String(mpu_acZ) + ",";
  dataString += String(mpu_temp) + ",";
  dataString += String(mpu_gyX) + ",";
  dataString += String(mpu_gyY) + ",";
  dataString += String(mpu_gyZ) + ",";
  dataString += String(gps_sats) + ",";
  dataString += String(gps_VDOP) + ",";
  dataString += String(gps_HDOP) + ",";
  dataString += String(gps_PDOP) + ",";
  dataString += String(gps_Latitude) + ",";
  dataString += String(gps_Longitude) + ",";
  dataString += String(gps_FixAge) + ",";
  dataString += String(gps_Date) + ",";
  dataString += String(gps_Time) + ",";
  dataString += String(gps_DateAge) + ",";
  dataString += String(gps_altitude) + ",";
  dataString += String(gps_Course) + ",";
  dataString += String(gps_Speed) + ",";
  dataString += String(gps_Card);
}

void dataString_to_SD()
{
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    DPRINTLN(dataString);
  }
  // if the file isn't open, pop up an error:
  else
  {
    DPRINTLN("error opening datalog.txt"); //*********************************************************************Achtung Fehlermeldung
  }
}
