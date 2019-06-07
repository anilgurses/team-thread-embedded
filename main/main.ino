#include <TinyGPS++.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <EnvironmentCalculations.h>


#define referencePressure  1016.3f // hPa local QFF (official meteor-station reading)
#define outdoorTemp  29.1f // °C  measured local outdoor temp.
#define barometerAltitude  50.3f


RTC_DS3231 rtc;
SoftwareSerial xBee(2, 3); // RX, TX
SoftwareSerial gyr(9, 10);

// The TinyGPS++ object
TinyGPSPlus gps;
BME280I2C bme;


int YPR[3];
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
int steps = 0;
float steps_old = 0;
float temp = 0;
float rps = 0;
unsigned long start_time = 0;
unsigned long end_time = 0;


File myLog;

// The serial connection to the GPS device


unsigned int pckCnt = 0;

/*States
  0: Started
  1: Calibration
  2: Calibrated and waiting for launch
  3: Launch
  4: Deploy from the rocket
  5: Released from container
  6: Landed and waiting for pickup
*/

unsigned int state = 0;
unsigned long gyTime ;

void setup() {


  Serial.begin(9600);

  xBee.begin(9600);

  Wire.begin();

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  pinMode(8 , INPUT_PULLUP);
  pinMode(6 , OUTPUT);
  pinMode(7 , OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);


  while (!bme.begin())
  {
    delay(1000);
  }



  // Calibration init



  if (!SD.begin(4)) {
    while (1);
  }

  gyr.begin(115200);

  delay(2000);

  gyr.write(0XA5);
  gyr.write(0X45);
  gyr.write(0XEA);


  xBee.println(F("Started"));
  delay(1000);
  gyTime = millis();
}


void loop() {

  DateTime now = rtc.now();
  DateTime future (now);





  //  if(state == 0){
  //    bool coming = true;
  //    String rec;
  //    while(coming){
  //      String tmpVal = xBee.readString();
  //
  //      if (tmpVal == "\n"){
  //        if(rec == "C"){
  //          //Calibration
  //          gyr.write(0XA5); // Calibration commands
  //          gyr.write(0X58);
  //          gyr.write(0xFD);
  //          state = 1;
  //        }
  //      }else{
  //        rec = tmpVal;
  //      }
  //    }
  //
  //
  //    if(state == 0){
  //      delay(500);
  //      return;
  //    }
  //
  //  }



  float temp(NAN), hum(NAN), pres(NAN);
  start_time = millis();
  end_time = start_time + 5;




  while (millis() < end_time)
  {
    if (digitalRead(8))
    {
      steps = steps + 1;
      while (digitalRead(8));
    }
  }
  
  temp = steps - steps_old;
  steps_old = steps;
  rps = (temp / 20) * 200;


  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;


  bme.read(pres, temp, hum, tempUnit, presUnit);
  float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);

  //<TEAM ID>,<MISSION TIME>,<PACKET COUNT>,<ALTITUDE>, <PRESSURE>, <TEMP>,<VOLTAGE>,<GPS TIME>,<GPS LATITUDE>,<GPS LONGITUDE>
  //,<GPS ALTITUDE>,<GPS SATS>,<PITCH>,<ROLL>,<BLADE SPIN RATE>,<SOFTWARE STATE>,<BONUS DIRECTION>
  int val = analogRead(A6);
  float voltage = val * (6.5 / 1023.0);

  serialEvent();


  if (Re_buf[0] == 0x5A && Re_buf[1] == 0x5A) {
    YPR[0] = (Re_buf[8] << 8 | Re_buf[9]) / 100;
    YPR[1] = (Re_buf[6] << 8 | Re_buf[7]) / 100; //PITCH
    YPR[2] = (Re_buf[4] << 8 | Re_buf[5]) / 100; //ROLL
  }
  unsigned long endGy = millis() + 1000;


  xBee.print(F("{"));
  xBee.print(F("2505"));
  xBee.print(F(","));
  xBee.print(future.hour(), DEC);
  xBee.print(F(":"));
  xBee.print(future.minute(), DEC);
  xBee.print(F(":"));
  xBee.print(future.second(), DEC);
  xBee.print(F(","));
  xBee.print(pckCnt);
  xBee.print(F(","));
  xBee.print(altitude);
  xBee.print(F(","));
  xBee.print(pres);
  xBee.print(F(","));
  xBee.print(temp);
  xBee.print(F(","));
  xBee.print(voltage);
  xBee.print(F(","));
  xBee.print(gps.time.hour());
  xBee.print(F(":"));
  xBee.print(gps.time.minute());
  xBee.print(F(":"));
  xBee.print(gps.time.second());
  xBee.print(F(","));
  xBee.print(gps.location.lat());
  xBee.print(F(","));
  xBee.print(gps.location.lng());
  xBee.print(F(","));
  xBee.print(gps.altitude.meters());
  xBee.print(F(","));
  xBee.print(gps.satellites.value());
  xBee.print(F(","));
  xBee.print(YPR[1], DEC);//PITCH
  xBee.print(F(","));
  xBee.print(YPR[2], DEC);//Roll
  xBee.print(F(","));
  xBee.print(rps);
  xBee.print(F(","));
  xBee.print(state);
  xBee.print(F(","));
  xBee.print(YPR[0], DEC);//Nadir
  xBee.print(F(","));
  xBee.print(F("}"));
  xBee.println();

  myLog = SD.open(F("log.txt"), FILE_WRITE);

  if (myLog) {
    myLog.print(F("{"));
    myLog.print(F("2505"));
    myLog.print(F(","));
    myLog.print(future.hour(), DEC);
    myLog.print(F(":"));
    myLog.print(future.minute(), DEC);
    myLog.print(F(":"));
    myLog.print(future.second(), DEC);
    myLog.print(F(","));
    myLog.print(pckCnt);
    myLog.print(F(","));
    myLog.print(altitude);
    myLog.print(F(","));
    myLog.print(pres);
    myLog.print(F(","));
    myLog.print(temp);
    myLog.print(F(","));
    myLog.print(voltage);
    myLog.print(F(","));
    myLog.print(gps.time.hour());
    myLog.print(F(":"));
    myLog.print(gps.time.minute());
    myLog.print(F(":"));
    myLog.print(gps.time.second());
    myLog.print(F(","));
    myLog.print(gps.location.lat());
    myLog.print(F(","));
    myLog.print(gps.location.lng());
    myLog.print(F(","));
    myLog.print(gps.altitude.meters());
    myLog.print(F(","));
    myLog.print(gps.satellites.value());
    myLog.print(F(","));
    myLog.print(YPR[1], DEC);//PITCH
    myLog.print(F(","));
    myLog.print(YPR[2], DEC);//Roll
    myLog.print(F(","));
    myLog.print(rps);
    myLog.print(F(","));
    myLog.print(state);
    myLog.print(F(","));
    myLog.print(YPR[0], DEC);//Nadir
    myLog.print(F(","));
    myLog.print(F("}"));
    myLog.println();
    myLog.close();
  } else {
    // if the file didn't open, print an error:
    xBee.println(F("error opening log.txt"));
  }

  pckCnt++;
  gyTime = millis();



  smartDelay(10);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}

void serialEvent() {
  Re_buf[counter] = (unsigned char)gyr.read();
  if (counter == 0 && Re_buf[0] != 0x5A) return;
  counter++;
  if (counter == 11)
  {
    counter = 0;
  }

}
