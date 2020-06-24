/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  This example demonstrates how to use alternate PT100 I2C addresses and Wire ports

  Please consult the Qwiic PT100 schematic for the address jumper settings

  The PT100 will be configured for 4-wire mode

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

  Wire1.begin(); // This example uses Wire1

  //mySensor.enableDebugging(); //Uncomment this line to enable debug messages on Serial

  if (mySensor.begin(0x44, Wire1) == false) //Connect to the PT100 using: Address 0x44 and the Wire1 port
  {
    Serial.println(F("Qwiic PT100 not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  mySensor.configureADCmode(ADS122C04_4WIRE_MODE); // Configure the PT100 for 4-wire mode

}

void loop()
{
  // Get the temperature in Centigrade
  float temperature = mySensor.readPT100Centigrade();

  // Print the temperature
  Serial.print(F("The temperature is: "));
  Serial.print(temperature);
  Serial.println(F("C"));

  delay(250); //Don't pound too hard on the I2C bus
}
