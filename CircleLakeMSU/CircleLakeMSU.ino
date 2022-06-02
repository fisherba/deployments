/** =========================================================================
 * @file logging_to_MMW.ino
 * @brief Example logging data and publishing to Monitor My Watershed.
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @copyright (c) 2017-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Atom with PlatformIO Core 5.1.1·Home 3.3.4
 *     Modular Sensors *RainCounterUpdate* branch,
 *     1 commit ahead of master v0.28.3 as of 2021-04-05
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for the Arduino IDE
//  NOTE:  These are ONLY needed to compile with the Arduino IDE.
//         If you use PlatformIO, you should set these build flags in your
//         platformio.ini
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif
/** End [defines] */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// To get all of the base classes for ModularSensors, include LoggerBase.
// NOTE:  Individual sensor definitions must be included separately.
#include <LoggerBase.h>
/** End [includes] */


// ==========================================================================
//  Creating Additional Serial Ports
// ==========================================================================
// The modem and a number of sensors communicate over UART/TTL - often called
// "serial". "Hardware" serial ports (automatically controlled by the MCU) are
// generally the most accurate and should be configured and used for as many
// peripherals as possible.  In some cases (ie, modbus communication) many
// sensors can share the same serial port.

// AltSoftSerial by Paul Stoffregen
// (https://github.com/PaulStoffregen/AltSoftSerial) is the most accurate
// software serial port for AVR boards. AltSoftSerial can only be used on one
// set of pins on each board so only one AltSoftSerial port can be used. Not all
// AVR boards are supported by AltSoftSerial.
/** Start [altsoftserial] */
#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;
/** End [altsoftserial] */

// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you need.
// Not all AVR boards are supported by NeoSWSerial.
/** Start [neoswserial] */
#include <NeoSWSerial.h>          // for the stream communication
const int8_t neoSSerial1Rx = 11;  // data in pin
const int8_t neoSSerial1Tx = -1;  // data out pin
NeoSWSerial  neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR() {
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}
/** End [neoswserial] */

// ==========================================================================
//  Assigning Serial Port Functionality
// ==========================================================================
/** Start [assign_ports_sw] */

// Define the serial port for modbus
// Modbus (at 9600 8N1) is used by the Keller level loggers and Yosemitech
// sensors
// Since AltSoftSerial is the best software option, we use it for modbus
// If AltSoftSerial (or its pins) aren't avaiable, use NeoSWSerial
// SoftwareSerial **WILL NOT** work for modbus!
#define modbusSerial altSoftSerial  // For AltSoftSerial
// #define modbusSerial neoSSerial1  // For Neo software serial
// #define modbusSerial softSerial1  // For software serial

// The Maxbotix sonar is the only sensor that communicates over a serial port
// but does not use modbus
// Since the Maxbotix only needs one-way communication and sends a simple text
// string repeatedly, almost any software serial port will do for it.
// #define sonarSerial altSoftSerial  // For AltSoftSerial
#define sonarSerial neoSSerial1     // For Neo software serial
// #define sonarSerial softSerial1  // For software serial

/** End [assign_ports_sw] */


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "CircleLakeMSU.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "Mayfly-20191";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15;
// Your logger's timezone.
const int8_t timeZone = -6;  // Central Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = A7;  // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */


// ==========================================================================
//  Wifi/Cellular Modem Options
// ==========================================================================
/** Start [xbee_cell_transparent] */
// For any Digi Cellular XBee's
// NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global) can be used
// in either bypass or transparent mode, each with pros and cons
// The Telit based Digi XBees (LTE Cat1) can only use this mode.
#include <modems/DigiXBeeCellularTransparent.h>

// Create a reference to the serial port for the modem
HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
const int32_t   modemBaud   = 9600;     // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
const int8_t modemVccPin    = -2;     // MCU pin controlling modem power
const int8_t modemStatusPin = 19;     // MCU pin used to read modem status
const bool useCTSforStatus  = false;  // Flag to use the XBee CTS pin for status
const int8_t modemResetPin  = 20;     // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;    // MCU pin for modem sleep/wake request
const int8_t modemLEDPin = redLED;    // MCU pin connected an LED to show modem
                                      // status (-1 if unconnected)

// Network connection information
const char* apn = "hologram";  // The APN for the gprs connection

// NOTE:  If possible, use the `STATUS/SLEEP_not` (XBee pin 13) for status, but
// the `CTS` pin can also be used if necessary
DigiXBeeCellularTransparent modemXBCT(&modemSerial, modemVccPin, modemStatusPin,
                                      useCTSforStatus, modemResetPin,
                                      modemSleepRqPin, apn);
// Create an extra reference to the modem by a generic name
DigiXBeeCellularTransparent modem = modemXBCT;
/** End [xbee_cell_transparent] */


// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */


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
//  Yosemitech Y511 Turbidity Sensor with Wiper
// ==========================================================================
/** Start [y511] */
#include <sensors/YosemitechY511.h>

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section
// Sensor Serial Number      0x01      ------    YL2920031803
byte         y511ModbusAddress = 0x03;  // The modbus address of the Y511
const int8_t y511AdapterPower =
    sensorPowerPin;  // RS485 adapter power pin (-1 if unconnected)
const int8_t  y511SensorPower = A3;  // Sensor power pin
const int8_t  y511EnablePin   = -1;  // Adapter RE/DE pin (-1 if not applicable)
const uint8_t y511NumberReadings = 5;
// The manufacturer recommends averaging 10 readings, but we take 5 to minimize
// power consumption

// Create a Y511-A Turbidity sensor object
YosemitechY511 y511(y511ModbusAddress, modbusSerial, y511AdapterPower,
                    y511SensorPower, y511EnablePin, y511NumberReadings);

/** End [y511] */

// ==========================================================================
//  Yosemitech Y520 Conductivity Sensor
// ==========================================================================
/** Start [y520] */
#include <sensors/YosemitechY520.h>

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section

byte         y520ModbusAddress = 0x01;  // The modbus address of the Y520
// Sensor Serial Number YL0920042801
const int8_t y520AdapterPower =
    sensorPowerPin;  // RS485 adapter power pin (-1 if unconnected)
const int8_t  y520SensorPower = A3;  // Sensor power pin
const int8_t  y520EnablePin   = -1;  // Adapter RE/DE pin (-1 if not applicable)
const uint8_t y520NumberReadings = 5;
// The manufacturer recommends averaging 10 readings, but we take 5 to minimize
// power consumption

// Create a Y520 conductivity sensor object
YosemitechY520 y520(y520ModbusAddress, modbusSerial, y520AdapterPower,
                    y520SensorPower, y520EnablePin, y520NumberReadings);

/** End [y520] */

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
//  Maxbotix HRXL Ultrasonic Range Finder
// ==========================================================================
/** Start [maxbotics] */
#include <sensors/MaxBotixSonar.h>

// A Maxbotix sonar with the trigger pin disconnect CANNOT share the serial port
// A Maxbotix sonar using the trigger may be able to share but YMMV

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section

const int8_t SonarPower = sensorPowerPin;  // Excite (power) pin
    // (-1 if unconnected)
const int8_t Sonar1Trigger = -1;  // Trigger pin
    // (a unique negative number if unconnected)
const uint8_t sonar1NumberReadings = 3;  // The number of readings to average

// Create a MaxBotix Sonar sensor object
MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger,
                     sonar1NumberReadings);

Variable* sonar1Range =
    new MaxBotixSonar_Range(&sonar1, "12345678-abcd-1234-ef00-1234567890ab");
/** End [maxbotics] */


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
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  Creating Data Publisher[s]
// ==========================================================================
/** Start [publishers] */
// A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char* registrationToken =
    "8c5936b3-2e7c-4e25-890a-b61552a1511d";  // Device registration token
const char* samplingFeature =
    "b22fb85d-e96d-4790-ac4c-a02ea190bd85";  // Sampling feature UUID

// Create a data publisher for the Monitor My Watershed/EnviroDIY POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient,
                                 registrationToken, samplingFeature);
/** End [publishers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}

// Reads the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
// Wait for USB connection to be established by PC
// NOTE:  Only use this when debugging - if not connected to a PC, this
// could prevent the script from starting
#if defined SERIAL_PORT_USBVIRTUAL
    while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)) {}
#endif

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

    /** Start [setup_softserial] */
    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt,
                        CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif
    /** End [setup_softserial] */

    /** Start [setup_serial_begins] */
    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Start the stream for the modbus sensors;
    // all currently supported modbus sensors use 9600 baud
    modbusSerial.begin(9600);

    // Start the SoftwareSerial stream for the sonar; it will always be at 9600
    // baud
    sonarSerial.begin(9600);
    /** End [setup_serial_begins] */


    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }

    #if defined MS_BUILD_TEST_XBEE_CELLULAR
        /** Start [setup_xbeec_carrier] */
        // Extra modem set-up
        Serial.println(F("Waking modem and setting Cellular Carrier Options..."));
        modem.modemWake();  // NOTE:  This will also set up the modem
        // Go back to command mode to set carrier options
        modem.gsmModem.commandMode();
        // Carrier Profile - 0 = Automatic selection
        //                 - 1 = No profile/SIM ICCID selected
        //                 - 2 = AT&T
        //                 - 3 = Verizon
        // NOTE:  To select T-Mobile, you must enter bypass mode!
        modem.gsmModem.sendAT(GF("CP"), 2);
        modem.gsmModem.waitResponse();
        // Cellular network technology - 0 = LTE-M with NB-IoT fallback
        //                             - 1 = NB-IoT with LTE-M fallback
        //                             - 2 = LTE-M only
        //                             - 3 = NB-IoT only
        // NOTE:  As of 2020 in the USA, AT&T and Verizon only use LTE-M
        // T-Mobile uses NB-IOT
        modem.gsmModem.sendAT(GF("N#"), 2);
        modem.gsmModem.waitResponse();
        // Write changes to flash and apply them
        Serial.println(F("Wait while applying changes..."));
        // Write changes to flash
        modem.gsmModem.writeChanges();
        // Reset the cellular component to ensure network settings are changed
        modem.gsmModem.sendAT(GF("!R"));
        modem.gsmModem.waitResponse(30000L);
        // Force reset of the Digi component as well
        // This effectively exits command mode
        modem.gsmModem.sendAT(GF("FR"));
        modem.gsmModem.waitResponse(5000L);
    /** End [setup_xbeec_carrier] */
    #endif


    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.55 || !dataLogger.isRTCSane()) {
        // Synchronize the RTC with NIST
        // This will also set up the modem
        dataLogger.syncRTC();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);
        // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(true);
        // true = wait for internal housekeeping after write
    }

    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [complex_loop] */
// Use this long loop when you want to do something special
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.

// We use this long to reset AltSoftSerial pins to LOW,
// to reduce RS485 adapter power bleed on sleep

void loop() {
    // Reset the watchdog
    dataLogger.watchDogTimer.resetWatchDog();

    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    // We're only doing anything at all if the battery is above 3.4V
    if (dataLogger.checkInterval() && getBatteryVoltage() > 3.4) {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;
        dataLogger.watchDogTimer.resetWatchDog();

        // Print a line to show new reading
        Serial.println(F("------------------------------------------"));
        // Turn on the LED to show we're taking a reading
        dataLogger.alertOn();
        // Power up the SD Card, but skip any waits after power up
        dataLogger.turnOnSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();

        // Turn on the modem to let it start searching for the network
        // Only turn the modem on if the battery at the last interval was high
        // enough
        // NOTE:  if the modemPowerUp function is not run before the
        // completeUpdate
        // function is run, the modem will not be powered and will not
        // return a signal strength reading.
        if (getBatteryVoltage() > 3.6) modem.modemPowerUp();

        // Start the stream for the modbus sensors, if your RS485 adapter bleeds
        // current from data pins when powered off & you stop modbus serial
        // connection with digitalWrite(5, LOW), below.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        altSoftSerial.begin(9600);

        // Do a complete update on the variable array.
        // This this includes powering all of the sensors, getting updated
        // values, and turing them back off.
        // NOTE:  The wake function for each sensor should force sensor setup
        // to run if the sensor was not previously set up.
        varArray.completeUpdate();

        dataLogger.watchDogTimer.resetWatchDog();

        // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
        // on sleep, because Modbus Stop bit leaves these pins HIGH.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        digitalWrite(5, LOW);  // Reset AltSoftSerial Tx pin to LOW
        digitalWrite(6, LOW);  // Reset AltSoftSerial Rx pin to LOW

        // Create a csv data record and save it to the log file
        dataLogger.logToSD();
        dataLogger.watchDogTimer.resetWatchDog();

        // Connect to the network
        // Again, we're only doing this if the battery is doing well
        if (getBatteryVoltage() > 3.55) {
            dataLogger.watchDogTimer.resetWatchDog();
            if (modem.connectInternet()) {
                dataLogger.watchDogTimer.resetWatchDog();
                // Publish data to remotes
                Serial.println(F("Modem connected to internet."));
                dataLogger.publishDataToRemotes();

                // Sync the clock at midnight
                dataLogger.watchDogTimer.resetWatchDog();
                if (Logger::markedEpochTime != 0 &&
                    Logger::markedEpochTime % 86400 == 0) {
                    Serial.println(F("Running a daily clock sync..."));
                    dataLogger.setRTClock(modem.getNISTTime());
                    dataLogger.watchDogTimer.resetWatchDog();
                    modem.updateModemMetadata();
                    dataLogger.watchDogTimer.resetWatchDog();
                }

                // Disconnect from the network
                modem.disconnectInternet();
                dataLogger.watchDogTimer.resetWatchDog();
            }
            // Turn the modem off
            modem.modemSleepPowerDown();
            dataLogger.watchDogTimer.resetWatchDog();
        }

        // Cut power from the SD card - without additional housekeeping wait
        dataLogger.turnOffSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();
        // Turn off the LED
        dataLogger.alertOff();
        // Print a line to show reading ended
        Serial.println(F("------------------------------------------\n"));

        // Unset flag
        Logger::isLoggingNow = false;
    }

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting) {
        // Start the stream for the modbus sensors, if your RS485 adapter bleeds
        // current from data pins when powered off & you stop modbus serial
        // connection with digitalWrite(5, LOW), below.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
    altSoftSerial.begin(9600);

        dataLogger.testingMode();
    }

    // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
    // on sleep, because Modbus Stop bit leaves these pins HIGH.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
    digitalWrite(5, LOW);  // Reset AltSoftSerial Tx pin to LOW
    digitalWrite(6, LOW);  // Reset AltSoftSerial Rx pin to LOW

    // Call the processor sleep
    dataLogger.systemSleep();
}
/** End [complex_loop] */
