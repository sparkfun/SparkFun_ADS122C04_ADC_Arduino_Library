/*
  Using the Qwiic PT100
  By: Paul Clark (PaulZC)
  Date: May 5th, 2020

  This example demonstrates how to read the raw voltage from the ADS122C04 AIN1 and AIN0 pins.

  The function readRawVoltage:
  - disables the IDAC current source
  - sets the gain to 1
  - configures the chip to use the internal 2.048V reference

  readRawVoltage returns a int32_t. The LSB is 2.048 / 2^23 = 0.24414 uV (0.24414 microvolts).

  If you want to configure the chip manually, see Example_ManualConfig.

  Make sure the PCB is configured for 4-wire mode (split pads A, B and C are open).
  Feed the differential voltage you want to measure onto terminals 2 & 3. +/-2.048V maximum!
  Terminal 2 is positive. Terminal 3 is negative (connect to 0V if required).

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

  mySensor.configureADCmode(ADS122C04_RAW_MODE); // Configure the PT100 for raw mode

}

void loop()
{
  // Get the raw voltage as int32_t
  int32_t raw_v = mySensor.readRawVoltage();

  // Convert to Volts (method 1)
  float volts_1 = ((float)raw_v) * 244.14e-9;

  // Convert to Volts (method 2)
  float volts_2 = ((float)raw_v) / 4096000;

  // Print the temperature and voltage
  Serial.print(F("The raw voltage is 0x"));
  Serial.print(raw_v, HEX);
  Serial.print(F("\t"));
  Serial.print(volts_1, 7); // Print the voltage with 7 decimal places
  Serial.print(F("V\t"));
  Serial.print(volts_2, 7); // Print the voltage with 7 decimal places
  Serial.println(F("V"));

  delay(250); //Don't pound the I2C bus too hard
}
