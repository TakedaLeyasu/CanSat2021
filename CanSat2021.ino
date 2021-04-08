//*************DEBUG****************

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
#define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DPRINT(...)     //now defines a blank line
#define DPRINTLN(...)   //now defines a blank line
#endif


//***********Bibliotheken***************

//SD-Datalogger
#include <SPI.h>
#include <SD.h>





const int _PIN_BMP_SDA = 2;
const int _PIN_BMP_SCL = 3;
const int _PIN_A_Temp = 4;
const int _PIN_GPS_RX = 6;
const int _PIN_GPS_TX = 8;
const int _PIN_CAM_EA = 7;
const int _PIN_Piper = 9;
const int _PIN_SD_CS = 10;
const int _PIN_LED = 18;
const int _PIN_DVR_record = 19;
//const int _PIN_LED2 = 20;
//const int _PIN_LED3 = 21;
/*
   SD card SPI bus:
   SCLK - pin 15
   MISO - pin 14
   MOSI - pin 16
   CS - siehe oben

*/


void setup() {
#ifdef DEBUG
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  setup_SD();

}

void loop() {
  // put your main code here, to run repeatedly:

}

//*************Funktionen***********
void setup_SD() {
  DPRINT("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(_PIN_SD_CS)) {
    DPRINTLN("Card failed, or not present");
    // don't do anything more:
    while (1); //********************************Achtung Fehlermeldung
  }
  DPRINTLN("card initialized.");



  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("start");
    dataFile.close();
    // print to the serial port too:
    DPRINTLN("start");
  }
  // if the file isn't open, pop up an error:
  else {
    DPRINTLN("error opening datalog.txt");
  }

}
