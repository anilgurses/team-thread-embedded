#include <SD.h> //Load SD library
#include <SFE_BMP180.h>
#include <Servo.h>
#include <Wire.h>

Servo myservo;
SFE_BMP180 pressure;
double baseline; // baseline pressure
int chipSelect = 4; //chip select pin for the MicroSD Card Adapter
int pos = 0;// variable to store the servo position
File file; // file object that is used to read and write data

void setup() {
  Serial.begin(9600); // start serial connection to print out debug messages and data
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.println("REBOOT");
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1); // Pause forever.
  }

  // Get the baseline pressure:
  
  baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb"); 
  
  
  pinMode(chipSelect, OUTPUT); // chip select pin must be set to OUTPUT mode
  
  if (!SD.begin(chipSelect)) { // Initialize SD card
    Serial.println("Could not initialize SD card."); // if return value is false, something went wrong.
  }
  
  if (SD.exists("file.txt")) { // if "file.txt" exists, fill will be deleted
    Serial.println("File exists.");
    if (SD.remove("file.txt") == true) {
      Serial.println("Successfully removed file.");
    } else {
      Serial.println("Could not removed file.");
    }
  }

  
}


void loop() {
  
  double a,P;
  int l=1;
  P = getPressure();
  a = pressure.altitude(P,baseline);
  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.println(" meters, ");
  Serial.print("altitude:  ");
  Serial.println(P);
  delay(500);
/*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position
  }
  */
 // if(a<=20 && a>=0){
    myservo.write(0);
    delay(500);
    myservo.write(180);
    delay(500);
    
   // }
  
  
  delay(1000);
  file = SD.open("file.txt", FILE_WRITE); //  "file.txt" to write data
  if (file) {
    file.println("Basinc:");
    file.println(P); // write number to file
    file.println("Yukseklik:");
    file.println(a);
    file.close(); // close file
    Serial.print("Wrote data"); // debug output: show written number in serial monitor
    
  } else {
    Serial.println("Could not open file (writing).");
  }
  
  file = SD.open("file.txt", FILE_READ); // open "file.txt" to read data
  if (file) {
    Serial.println("--- Reading start ---");
    char character;
    while ((character = file.read()) != -1) { // this while loop reads data stored in "file.txt" and prints it to serial monitor
      Serial.print(character);
    }
    file.close();
    Serial.println("--- Reading end ---");
  } else {
    Serial.println("Could not open file (reading).");
  }
  
  delay(5000); // wait for 5000ms
}


double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
