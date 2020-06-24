/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  This example demonstrates how to put the ADS122C04 into power down mode

  Hardware Connections:
  Plug a Qwiic cable into the PT100 and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>

#include <SparkFun_ADS122C04_ADC_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C0

SFE_ADS122C04 mySensor;

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("Qwiic PT100 Example"));

  Wire.begin();

  //mySensor.enableDebugging(); //Uncomment this line to enable debug messages on Serial

  if (mySensor.begin() == false) //Connect to the PT100 using the defaults: Address 0x45 and the Wire port
  {
    Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  mySensor.configureADCmode(ADS122C04_4WIRE_MODE); // Configure the PT100 for 4-wire mode

}

void loop()
{
  // Get the temperature in Centigrade - this also wakes up the ADS122C04 (by doing a start)
  float temperature = mySensor.readPT100Centigrade();

  // Print the temperature
  Serial.print(F("The temperature is: "));
  Serial.print(temperature);
  Serial.println(F("C"));

  // Put the ADS122C04 into power down mode
  Serial.println(F("Putting the ADS122C04 into power down mode..."));

  mySensor.powerdown();

  delay(5000); //Repeat every 5 seconds
}
