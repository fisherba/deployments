/*****************************************************************************
logging_to_EnviroDIY.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO 3.2.1
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team

This sketch is an example of logging data to an SD card and sending the data to
the EnviroDIY data portal.

; Using ModularSensors *calcVar* branch as of June 4, 2018: head, 52 commits ahead of 0.11.6
    https://github.com/EnviroDIY/ModularSensors.git#4a10279e1031513391ce03d1914684ce5db812ff

DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

// Select your modem chip, comment out all of the others
#define TINY_GSM_MODEM_SIM800  // Select for a SIM800, SIM900, or variant thereof
// #define TINY_GSM_MODEM_A6  // Select for a AI-Thinker A6 or A7 chip
// #define TINY_GSM_MODEM_M590  // Select for a Neoway M590
// #define TINY_GSM_MODEM_UBLOX  // Select for most u-blox cellular modems
// #define TINY_GSM_MODEM_ESP8266  // Select for an ESP8266 using the DEFAULT AT COMMAND FIRMWARE
// #define TINY_GSM_MODEM_XBEE  // Select for Digi brand WiFi or Cellular XBee's

// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts
// #include <LoggerEnviroDIY.h>


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The name of this file
const char *sketchName = "CircleLake2022.ino";

// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Mayfly_30352";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15;
// Your logger's timezone.
const int8_t timeZone = -6;  // Central Standard Time (CST=-6)


// ==========================================================================
//    Primary Arduino-Based Board and Processor
// ==========================================================================
#include <ProcessorStats.h>

const long serialBaud = 115200;  // Baud rate for the primary serial port for debugging
const int8_t greenLED = 8;  // Pin for the green LED (-1 if unconnected)
const int8_t redLED = 9;  // Pin for the red LED (-1 if unconnected)
const int8_t buttonPin = 21;  // Pin for a button to use to enter debugging mode (-1 if unconnected)
const int8_t wakePin = A7;  // Interrupt/Alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPin = 12;  // SD Card Chip Select/Slave Select Pin (must be defined!)

// Create and return the processor "sensor"
const char *MFVersion = "v0.5b";
ProcessorStats mayfly(MFVersion) ;


// ==========================================================================
//    Modem/Internet connection options
// ==========================================================================
HardwareSerial &ModemSerial = Serial1; // The serial port for the modem - software serial can also be used.

#if defined(TINY_GSM_MODEM_XBEE)
const long ModemBaud = 9600;  // Default for XBee is 9600, I've sped mine up to 57600
const int8_t modemSleepRqPin = 23;  // Modem SleepRq Pin (for sleep requests) (-1 if unconnected)
const int8_t modemStatusPin = 19;   // Modem Status Pin (indicates power status) (-1 if unconnected)
const int8_t modemVCCPin = -1;  // Modem power pin, if it can be turned on or off (-1 if unconnected)
ModemSleepType ModemSleepMode = modem_sleep_reverse;  // How the modem is put to sleep

#elif defined(TINY_GSM_MODEM_ESP8266)
const long ModemBaud = 57600;  // Default for ESP8266 is 115200, but the Mayfly itself stutters above 57600
const int8_t modemSleepRqPin = 19;  // Modem SleepRq Pin (for sleep requests) (-1 if unconnected)
const int8_t modemStatusPin = -1;   // Modem Status Pin (indicates power status) (-1 if unconnected)
const int8_t modemVCCPin = -1;  // Modem power pin, if it can be turned on or off (-1 if unconnected)
ModemSleepType ModemSleepMode = modem_always_on;  // How the modem is put to sleep

#elif defined(TINY_GSM_MODEM_UBLOX)
const long ModemBaud = 9600;  // SARA-U201 default seems to be 9600
const int8_t modemSleepRqPin = 23;  // Modem SleepRq Pin (for sleep requests) (-1 if unconnected)
const int8_t modemStatusPin = 19;   // Modem Status Pin (indicates power status) (-1 if unconnected)
const int8_t modemVCCPin = -1;  // Modem power pin, if it can be turned on or off (-1 if unconnected)
ModemSleepType ModemSleepMode = modem_sleep_held;  // How the modem is put to sleep

#else
const long ModemBaud = 9600;  // SIM800 auto-detects, but I've had trouble making it fast (19200 works)
const int8_t modemSleepRqPin = 23;  // Modem SleepRq Pin (for sleep requests) (-1 if unconnected)
const int8_t modemStatusPin = 19;   // Modem Status Pin (indicates power status) (-1 if unconnected)
const int8_t modemVCCPin = -1;  // Modem power pin, if it can be turned on or off (-1 if unconnected)
ModemSleepType ModemSleepMode = modem_sleep_held;  // How the modem is put to sleep
// Use "modem_sleep_held" if the DTR pin is held HIGH to keep the modem awake, as with a Sodaq GPRSBee rev6.
// Use "modem_sleep_pulsed" if the DTR pin is pulsed high and then low to wake the modem up, as with an Adafruit Fona or Sodaq GPRSBee rev4.
// Use "modem_sleep_reverse" if the DTR pin is held LOW to keep the modem awake, as with all XBees.
// Use "modem_always_on" if you do not want the library to control the modem power and sleep or if none of the above apply.
#endif

const char *apn = "apn.konekt.io";  // The APN for the gprs connection, unnecessary for WiFi
const char *wifiId = "xxxxx";  // The WiFi access point, unnecessary for gprs
const char *wifiPwd = "xxxxx";  // The password for connecting to WiFi, unnecessary for gprs

// Create the loggerModem instance
// A "loggerModem" is a combination of a TinyGSM Modem, a TinyGSM Client, and an on/off method
loggerModem modem;


// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <MaximDS3231.h>
// Create and return the DS3231 sensor object
MaximDS3231 ds3231(1);


// ==========================================================================
//    Maxbotix HRXL Ultrasonic Range Finder
// ==========================================================================

// Set up a serial port for receiving sonar data - in this case, using software serial
// Because the standard software serial library uses interrupts that conflict
// with several other libraries used within this program, we must use a
// version of software serial that has been stripped of interrupts and define
// the interrrupts for it using the enableInterrup library.

// If enough hardware serial ports are available on your processor, you should
// use one of those instead.  If the proper pins are avaialbe, AltSoftSerial
// by Paul Stoffregen is also superior to SoftwareSerial for this sensor.
// Neither hardware serial nor AltSoftSerial require any modifications to
// deal with interrupt conflicts.

const int SonarData = 11;     // data receive pin

#include <SoftwareSerial_ExtInts.h>  // for the stream communication
SoftwareSerial_ExtInts sonarSerial(SonarData, -1);  // No Tx pin is required, only Rx

// #include <NeoSWSerial.h>  // for the stream communication
// NeoSWSerial sonarSerial(SonarData, -1);  // No Tx pin is required, only Rx
// void NeoSWSISR()
// {
//   NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort( SonarData ) ) );
// }

#include <MaxBotixSonar.h>
const int8_t SonarPower = sensorPowerPin;  // Excite (power) pin
    // (-1 if unconnected)
const int8_t Sonar1Trigger = -1;  // Trigger pin (a negative number if unconnected) (A1 = 25)
// const int8_t Sonar2Trigger = A2;  // Trigger pin (a negative number if unconnected) (A2 = 26)
const uint8_t sonar1NumberReadings = 5;  // The number of readings to average
// Create and return the MaxBotix Sonar sensor object
MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger, sonar1NumberReadings) ;
// MaxBotixSonar sonar2(sonarSerial, SonarPower, Sonar2Trigger) ;

Variable* sonar1Range =
    new MaxBotixSonar_Range(&sonar1, "12345678-abcd-1234-ef00-1234567890ab");

// Set up a serial port for modbus communication - in this case, using AltSoftSerial
#include <AltSoftSerial.h>
AltSoftSerial modbusSerial;

// ==========================================================================
//    Yosemitech Y511 Turbidity Sensor with Wiper
// ==========================================================================
#include <YosemitechY511.h>
byte y511modbusAddress = 0x03;  // The modbus address of the Y511
const int8_t modbusPower = 22;  // Pin to switch power on and off (-1 if unconnected)
const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
const uint8_t y511NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption
// Create and return the Y511-A Turbidity sensor object
YosemitechY511 y511(y511modbusAddress, modbusSerial, modbusPower, max485EnablePin, y511NumberReadings);

// Create a Y511-A Turbidity sensor object
YosemitechY511 y511(y511ModbusAddress, modbusSerial, y511AdapterPower,
                    y511SensorPower, y511EnablePin, y511NumberReadings);

// ==========================================================================
//    Yosemitech Y520 Conductivity Sensor
// ==========================================================================
#include <YosemitechY520.h>
byte y520modbusAddress = 0x01;  // The modbus address of the Y520 Conductivity
// const int8_t modbusPower = 22;  // Pin to switch power on and off (-1 if unconnected)
// const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
const uint8_t y520NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption
// Create and return the Y520 conductivity sensor object
YosemitechY520 y520(y520modbusAddress, modbusSerial, modbusPower, max485EnablePin, y520NumberReadings);

// Create a Y520 conductivity sensor object
YosemitechY520 y520(y520ModbusAddress, modbusSerial, y520AdapterPower,
                    y520SensorPower, y520EnablePin, y520NumberReadings);

// ==========================================================================
//  External I2C Rain Tipping Bucket Counter
// ==========================================================================
/** Start [i2c_rain] */
#include <sensors/RainCounterI2C.h>

const uint8_t RainCounterI2CAddress = 0x08;
// I2C Address for EnviroDIY external tip counter; 0x08 by default
const float depthPerTipEvent = 0.01;  // rain depth in *inches* per tip event

// Create a Rain Counter sensor object
RainCounterI2C    tbi2c(RainCounterI2CAddress, depthPerTipEvent);
/** End [i2c_rain] */


// ==========================================================================
//  External I2C Rain Tipping Bucket Counter for wind, connected to an anemometer
// ==========================================================================
/** Start [i2c_rain_wind] */
#include <sensors/RainCounterI2C.h>

const uint8_t RainCounterI2CAddress2 = 0x06;
// I2C Address for EnviroDIY external tip counter; 0x08 by default
// const float depthPerTipEvent = 0.2;  // rain depth in mm per tip event

// Create a Rain Counter sensor object
RainCounterI2C    tbi2c2(RainCounterI2CAddress2, depthPerTipEvent);

// Create number of tips and rain depth variable pointers for the tipping bucket
Variable* tbi2cWindCounts =
    new RainCounterI2C_Tips(&tbi2c2, "12345678-abcd-1234-ef00-1234567890ab");
// Variable* tbi2cDepth =
//     new RainCounterI2C_Depth(&tbi2c, "12345678-abcd-1234-ef00-1234567890ab");
/** End [i2c_rain_wind] */


// ==========================================================================
//  Bosch BME280 Environmental Sensor
// ==========================================================================
/** Start [bme280] */
#include <sensors/BoschBME280.h>

const int8_t I2CPower    = sensorPowerPin;  // Power pin (-1 if unconnected)
uint8_t      BMEi2c_addr = 0x77;
// The BME280 can be addressed either as 0x77 (Adafruit default) or 0x76 (Grove
// default) Either can be physically mofidied for the other address

// Create a Bosch BME280 sensor object
BoschBME280 bme280(I2CPower, BMEi2c_addr);
/** End [bme280] */



// ==========================================================================
//    Calculated Variables
// ==========================================================================

// Create the function to calculate water level / gage height variable
float calculateSonarGageHeight(void)
{
    float sonarGageHeight = -9999;  // Always safest to start with a bad value
    float sonarGageHeight_mm = -9999;  // Always safest to start with a bad value
    const float minimumRange = 500;    // in millimeters
    const float maximumRange = 9999;    // in millimeters
    const float sonarDistanceToZeroStage = 304.8*(96.5/12); // in millimeters, where 304.8 mm = 1.00 ft
    float sonarDistanceMeasured = sonar1Range->getValue();
    if (sonarDistanceMeasured != -9999)  // make sure all inputs are good
    {
        sonarGageHeight_mm = sonarDistanceToZeroStage - sonarDistanceMeasured;
        sonarGageHeight = sonarGageHeight_mm / 304.8; // to convert to feet, divide by 304.8, or divide by 1 to remain in mm.
    }
    return sonarGageHeight;
}

// Properties of the calculated water level / gage height variable
const uint8_t sonarGageHeightVarResolution = 3;  // The number of digits after the decimal place
const char *sonarGageHeightVarName = "gageHeight";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *sonarGageHeightVarUnit = "Foot";  // This must be a value from http://vocabulary.odm2.org/units/
const char *sonarGageHeightVarCode = "SonarGageHeight";  // A short code for the variable
const char *sonarGageHeightVarUUID = "2db61932-df20-4dc5-a353-015dfeb9b178";

// Create the calculated water pressure variable pointer and return a variable pointer to it
Variable *calculatedSonarGageHeight = new Variable(calculateSonarGageHeight, sonarGageHeightVarResolution,
                                        sonarGageHeightVarName, sonarGageHeightVarUnit,
                                        sonarGageHeightVarCode, sonarGageHeightVarUUID);

// ==========================================================================

// Create the function to calculate wind speed variable
float calculateWindSpeed(void) {
    float windSpeed = -9999;  // Always safest to start with a bad value
    float period = -9999;  // seconds between gettting event counts
    float frequency = -9999;  // average event frequency in Hz
    float eventCount = tbi2cWindCounts->getValue();
    if (eventCount != -9999)  // make sure both inputs are good
    {
        period = loggingInterval * 60.0;    // in seconds
        frequency = eventCount/period; // average event frequency in Hz
        windSpeed = frequency * 2.5 * 1.60934;  // in km/h,
        // 2.5 mph/Hz & 1.60934 kmph/mph and 2.5 mph/Hz conversion factor from
    	// https://www.store.inspeed.com/Inspeed-Version-II-Reed-Switch-Anemometer-Sensor-Only-WS2R.htm
    }
    return windSpeed;
}

// Properties of the calculated variable
// The number of digits after the decimal place
const uint8_t calculatedVarResolution = 3;
// This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarName = "windSpeed";
// This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarUnit = "KilometerPerHour";
// A short code for the variable
const char *calculatedVarCode = "WindSpeed";
// The (optional) universallly unique identifier
const char *calculatedVarUUID = "1cfdccc0-9d09-4367-9705-1892731fb392";

// Create a calculated variable pointer and return a variable pointer to it
Variable *calculatedWindSpeed = new Variable(
    calculateWindSpeed, calculatedVarResolution, calculatedVarName,
    calculatedVarUnit, calculatedVarCode, calculatedVarUUID);


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
    // new ProcessorStats_SampleNumber(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab"),
    new BoschBME280_Temp(&bme280, "6575ae8b-fd62-4a76-b4f8-fc54b2411d89"),
    new BoschBME280_Humidity(&bme280, "9cb5d227-fa1f-4ab4-8f6f-997b86324164"),
    new BoschBME280_Pressure(&bme280, "77da2fd7-ff62-4666-988f-4a98551029fe"),
    new YosemitechY511_Turbidity(&y511, "0b5fae1c-4303-4d20-bbc9-3836f8441dc4"),
    // new YosemitechY511_Temp(&y511, "12345678-abcd-1234-ef00-1234567890ab"),
    new YosemitechY520_Cond(&y520, "84f37fdd-2852-4802-9022-aaeb78b6dc92"),
    new YosemitechY520_Temp(&y520, "c7fc006f-65d9-49ff-b6de-8484a236e8ce"),
    // new RainCounterI2C_Tips(&tbi2c, "12345678-abcd-1234-ef00-1234567890ab"),
    new RainCounterI2C_Depth(&tbi2c, "29a546cc-a91e-45fd-b660-8c61509f5e43"),
    sonar1Range,
    calculatedSonarGageHeight,
    tbi2cWindCounts,
    calculatedWindSpeed,
    new ProcessorStats_Battery(&mcuBoard, "989621c4-f6a9-4310-9b00-867c6ad6a8b0"),
    // new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
};

// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);
// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
// Create a new logger instance
LoggerEnviroDIY EnviroDIYLogger(LoggerID, loggingInterval, sdCardPin, wakePin, &varArray);


// ==========================================================================
// Device registration and sampling feature information
//   This should be obtained after registration at http://data.envirodiy.org
// ==========================================================================
const char *registrationToken = "8c5936b3-2e7c-4e25-890a-b61552a1511d";   // Device registration token
const char *samplingFeature = "b22fb85d-e96d-4790-ac4c-a02ea190bd85";     // Sampling feature UUID


// ==========================================================================
//    Working Functions
// ==========================================================================

// Flashes the LED's on the primary board
void greenredflash(int numFlash = 4, int rate = 75)
{
  for (int i = 0; i < numFlash; i++) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    delay(rate);
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
    delay(rate);
  }
  digitalWrite(redLED, LOW);
}


// ==========================================================================
// Main setup function
// ==========================================================================
void setup()
{
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Start the serial connection with the modem
    ModemSerial.begin(ModemBaud);

    // Start the stream for the modbus sensors
    modbusSerial.begin(9600);

    // Start the SoftwareSerial stream for the sonar
    sonarSerial.begin(9600);
    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(SonarData, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(SonarData, NeoSWSISR, CHANGE);
    #endif

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);

    // Set the timezone and offsets
    // Logging in the given time zone
    Logger::setTimeZone(timeZone);
    // Offset is the same as the time zone because the RTC is in UTC
    Logger::setTZOffset(timeZone);

    // Setup the logger modem
    #if defined(TINY_GSM_MODEM_ESP8266)
        modem.setupModem(&ModemSerial, modemVCCPin, modemStatusPin, modemSleepRqPin, ModemSleepMode, wifiId, wifiPwd);
    #elif defined(TINY_GSM_MODEM_XBEE)
        modem.setupModem(&ModemSerial, modemVCCPin, modemStatusPin, modemSleepRqPin, ModemSleepMode, wifiId, wifiPwd);
        // modem.setupModem(&ModemSerial, modemVCCPin, modemStatusPin, modemSleepRqPin, ModemSleepMode, apn);
    #else
        modem.setupModem(&ModemSerial, modemVCCPin, modemStatusPin, modemSleepRqPin, ModemSleepMode, apn);
    #endif

    // Attach the modem and information pins to the logger
    EnviroDIYLogger.attachModem(modem);
    EnviroDIYLogger.setAlertPin(greenLED);
    EnviroDIYLogger.setTestingModePin(buttonPin);

    // Enter the tokens for the connection with EnviroDIY
    EnviroDIYLogger.setToken(registrationToken);
    EnviroDIYLogger.setSamplingFeatureUUID(samplingFeature);

    // Begin the logger
    EnviroDIYLogger.begin();

    // Reset AltSoftSerial pins to LOW, to reduce power bleed on sleep,
    // because Modbus Stop bit leaves these pins HIGH
    digitalWrite(5, LOW);   // Reset AltSoftSerial Tx pin to LOW
    digitalWrite(6, LOW);   // Reset AltSoftSerial Rx pin to LOW

    // Blink the LEDs really fast to show start-up is done
    greenredflash(6, 25);
}


// ==========================================================================
// Main loop function
// ==========================================================================
void loop()
{
    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    if (EnviroDIYLogger.checkInterval())
    {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;

        // Print a line to show new reading
        Serial.print(F("------------------------------------------\n"));
        // Turn on the LED to show we're taking a reading
        digitalWrite(greenLED, HIGH);

        // Turn on the modem to let it start searching for the network
        modem.modemPowerUp();

        // Start the stream for the modbus sensors
        // Because RS485 adapters tend to "steal" current from the data pins
        // we will explicitly start and end the serial connection in the loop.
        modbusSerial.begin(9600);

        // Send power to all of the sensors (do this directly on the VariableArray)
        Serial.print(F("Powering sensors...\n"));
        varArray.sensorsPowerUp();
        // Wake up all of the sensors (do this directly on the VariableArray)
        Serial.print(F("Waking sensors...\n"));
        varArray.sensorsWake();
        // Update the values from all attached sensors (do this directly on the VariableArray)
        Serial.print(F("Updating sensor values...\n"));
        varArray.updateAllSensors();
        // Put sensors to sleep (do this directly on the VariableArray)
        Serial.print(F("Putting sensors back to sleep...\n"));
        varArray.sensorsSleep();
        // Cut sensor power (do this directly on the VariableArray)
        Serial.print(F("Cutting sensor power...\n"));
        varArray.sensorsPowerDown();

        // Reset AltSoftSerial pins to LOW, to reduce power bleed on sleep,
        // because Modbus Stop bit leaves these pins HIGH
        digitalWrite(5, LOW);   // Reset AltSoftSerial Tx pin to LOW
        digitalWrite(6, LOW);   // Reset AltSoftSerial Rx pin to LOW

        // Connect to the network
        Serial.print(F("Connecting to the internet...\n"));
        if (modem.connectInternet())
        {
            // Post the data to the WebSDL
            EnviroDIYLogger.postDataEnviroDIY();

            // Once a day, at midnight, sync the clock
            if (Logger::markedEpochTime % 86400 == 0)
            {
                // Synchronize the RTC (the loggers have the same clock, pick one)
                EnviroDIYLogger.syncRTClock(modem.getNISTTime());
            }
            // Disconnect from the network
            modem.disconnectInternet();
        }
        // Turn the modem off
        modem.modemPowerDown();

        // Create a csv data record and save it to the log file
        Serial.print(F("\nWriting to the SD card...\n"));
        EnviroDIYLogger.logToSD();

        // Turn off the LED
        digitalWrite(greenLED, LOW);
        // Print a line to show reading ended
        Serial.print(F("------------------------------------------\n\n"));
    }


    // Check if it was instead the testing interrupt that woke us up
    // Want to enter the testing mode for the "complete" logger so we can see
    // the data from _ALL_ sensors
    // NOTE:  The testingISR attached to the button at the end of the "setup()"
    // function turns on the startTesting flag.  So we know if that flag is set
    // then we want to run the testing mode function.

    // Start the stream for the modbus sensors
    // Because RS485 adapters tend to "steal" current from the data pins
    // we will explicitly start and end the serial connection in the loop.
    modbusSerial.begin(9600);

    if (Logger::startTesting) EnviroDIYLogger.testingMode();

    // Reset AltSoftSerial pins to LOW, to reduce power bleed on sleep,
    // because Modbus Stop bit leaves these pins HIGH
    digitalWrite(5, LOW);   // Reset AltSoftSerial Tx pin to LOW
    digitalWrite(6, LOW);   // Reset AltSoftSerial Rx pin to LOW


    // Call the processor sleep
    // Only need to do this for one of the loggers
    EnviroDIYLogger.systemSleep();
}
