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
#include <LoggerEnviroDIY.h>


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The name of this file
const char *sketchName = "RPB-4.ino";

// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "RPB-4_Mayfly_170289";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 10;
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
const char *MFVersion = "v0.5";
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
const int8_t SonarPower = 22;  // Excite (power) pin (-1 if unconnected)
const int8_t Sonar1Trigger = -1;  // Trigger pin (a negative number if unconnected) (A1 = 25)
// const int8_t Sonar2Trigger = A2;  // Trigger pin (a negative number if unconnected) (A2 = 26)
const uint8_t sonar1NumberReadings = 5;  // The number of readings to average
// Create and return the MaxBotix Sonar sensor object
MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger, sonar1NumberReadings) ;
// MaxBotixSonar sonar2(sonarSerial, SonarPower, Sonar2Trigger) ;


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


// ==========================================================================
//    The array that contains all variables to be logged
// ==========================================================================
// Create pointers for all of the variables from the sensors
// at the same time putting them into an array
Variable *variableList[] = {
    // new ApogeeSQ212_PAR(&SQ212, "12345678-abcd-1234-efgh-1234567890ab"),
    // new AOSongAM2315_Humidity(&am2315, "12345678-abcd-1234-efgh-1234567890ab"),
    // new AOSongAM2315_Temp(&am2315, "12345678-abcd-1234-efgh-1234567890ab"),
    // new AOSongDHT_Humidity(&dht, "12345678-abcd-1234-efgh-1234567890ab"),
    // new AOSongDHT_Temp(&dht, "12345678-abcd-1234-efgh-1234567890ab"),
    // new AOSongDHT_HI(&dht, "12345678-abcd-1234-efgh-1234567890ab"),
    // new BoschBME280_Temp(&bme280, "12345678-abcd-1234-efgh-1234567890ab"),
    // new BoschBME280_Humidity(&bme280, "12345678-abcd-1234-efgh-1234567890ab"),
    // new BoschBME280_Pressure(&bme280, "12345678-abcd-1234-efgh-1234567890ab"),
    // new BoschBME280_Altitude(&bme280, "12345678-abcd-1234-efgh-1234567890ab"),
    // new CampbellOBS3_Turbidity(&osb3low, "12345678-abcd-1234-efgh-1234567890ab", "TurbLow"),
    // new CampbellOBS3_Turbidity(&osb3high, "12345678-abcd-1234-efgh-1234567890ab", "TurbHigh"),
    // new Decagon5TM_Ea(&fivetm, "12345678-abcd-1234-efgh-1234567890ab"),
    // new Decagon5TM_Temp(&fivetm, "12345678-abcd-1234-efgh-1234567890ab"),
    // new Decagon5TM_VWC(&fivetm, "12345678-abcd-1234-efgh-1234567890ab"),
    // new DecagonCTD_Cond(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    // new DecagonCTD_Temp(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    // new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    // new DecagonES2_Cond(&es2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new DecagonES2_Temp(&es2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new ExternalVoltage_Volt(&extvolt, "12345678-abcd-1234-efgh-1234567890ab"),
    new MaxBotixSonar_Range(&sonar1, "3a2de436-7468-4c5d-a88a-f079e5ad7e7d"),
    // new MaxBotixSonar_Range(&sonar2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MaximDS18_Temp(&ds18_1, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MaximDS18_Temp(&ds18_2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MaximDS18_Temp(&ds18_3, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MaximDS18_Temp(&ds18_4, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MaximDS18_Temp(&ds18_5, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MeaSpecMS5803_Temp(&ms5803, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MeaSpecMS5803_Pressure(&ms5803, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MPL115A2_Temp(&mpl115a2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new MPL115A2_Pressure(&mpl115a2, "12345678-abcd-1234-efgh-1234567890ab"),
    // new RainCounterI2C_Tips(&tip, "12345678-abcd-1234-efgh-1234567890ab"),
    // new RainCounterI2C_Depth(&tip, "12345678-abcd-1234-efgh-1234567890ab"),
    // new KellerAcculevel_Pressure(&acculevel, "12345678-abcd-1234-efgh-1234567890ab"),
    // new KellerAcculevel_Temp(&acculevel, "12345678-abcd-1234-efgh-1234567890ab"),
    // new KellerAcculevel_Height(&acculevel, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY504_DOpct(&y504, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY504_Temp(&y504, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY504_DOmgL(&y504, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY510_Temp(&y510, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY510_Turbidity(&y510, "12345678-abcd-1234-efgh-1234567890ab"),
    new YosemitechY511_Temp(&y511, "523bc735-ebca-47f2-a81e-38b1af23b658"),
    new YosemitechY511_Turbidity(&y511, "72b20e85-cc44-48a1-a384-d28901e38113"),
    // new YosemitechY514_Temp(&y514, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY514_Chlorophyll(&y514, "12345678-abcd-1234-efgh-1234567890ab"),
    new YosemitechY520_Temp(&y520, "c7b8ca07-a214-47c9-8fe5-d47a601f49ee"),
    new YosemitechY520_Cond(&y520, "0e46b047-2675-4697-82f1-e9eab2facb74"),
    // new YosemitechY532_Temp(&y532, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY532_Voltage(&y532, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY532_pH(&y532, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_DOmgL(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_Turbidity(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_Cond(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_pH(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_Temp(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_ORP(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_Chlorophyll(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YosemitechY4000_BGA(&y4000, "12345678-abcd-1234-efgh-1234567890ab"),
    // new ZebraTechDOpto_Temp(&dopto, "12345678-abcd-1234-efgh-1234567890ab"),
    // new ZebraTechDOpto_DOpct(&dopto, "12345678-abcd-1234-efgh-1234567890ab"),
    // new ZebraTechDOpto_DOmgL(&dopto, "12345678-abcd-1234-efgh-1234567890ab"),
    // new ProcessorStats_FreeRam(&mayfly, "63d202e8-5294-4aa0-a4aa-5c57a62aa428"),
    new ProcessorStats_Batt(&mayfly, "8f79d053-b990-4533-a6ee-513169803eec"),
    // new MaximDS3231_Temp(&ds3231, "e27eb08f-42ba-45e3-be52-5d946f7a5772"),
    // new Modem_RSSI(&modem, "12345678-abcd-1234-efgh-1234567890ab"),
    // new Modem_SignalPercent(&modem, "12345678-abcd-1234-efgh-1234567890ab"),
    // new YOUR_variableName_HERE(&)
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
const char *registrationToken = "d86bb6c7-e0c4-4c19-acb7-17a4c7f3eeca";   // Device registration token
const char *samplingFeature = "103b2e3e-2c53-4a39-8d76-3c2f0a016615";     // Sampling feature UUID


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
