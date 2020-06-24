/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  This example demonstrates how to read high temperatures in Centigrade
  using 4-wire mode.

  At temperatures below 270C, the gain of the ADS122C04 can be set to 8
  to provide maximal resolution.

  Between 270C and 850C, the gain needs to be reduced to 4 to avoid
  saturating the ADC.

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

  mySensor.configureADCmode(ADS122C04_4WIRE_HI_TEMP); // Configure the PT100 for high temperature 4-wire mode
  //mySensor.configureADCmode(ADS122C04_3WIRE_HI_TEMP); // Uncomment this line to configure the PT100 for high temperature 3-wire mode
  //mySensor.configureADCmode(ADS122C04_2WIRE_HI_TEMP); // Uncomment this line to configure the PT100 for high temperature 2-wire mode
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
