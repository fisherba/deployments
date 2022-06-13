# Deployment Notes

#### SiteCode: RPB-4
Site Link: http://data.envirodiy.org/sites/RPB-4/

Deployed SNs: See [RPB-4 record](https://docs.google.com/spreadsheets/d/1Qr-Epi9pFq7NtADK5qHTS2Ip3TvbOFoG_Seqt_cN4U4/edit#gid=1917587329&range=5:5) in [Limno-EnviroDIY_Deployed-SerialNumbers](https://docs.google.com/spreadsheets/d/1Qr-Epi9pFq7NtADK5qHTS2Ip3TvbOFoG_Seqt_cN4U4/edit#gid=1917587329) Google Sheet

#### Library Dependencies (in PlatformIO.ini)

```.ini
lib_deps =
; Using ModularSensors *master* branch as of May 18, 2018: head, = 0.11.7
    EnviroDIY_ModularSensors@>=0.11.7
    https://github.com/PaulStoffregen/AltSoftSerial.git
    https://github.com/EnviroDIY/SoftwareSerial_ExternalInts.git
```

#### Logger Settings

```C++
// The name of this file
const char *sketchName = "RPB-4.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "RPB-4_Mayfly_170289";
// How frequently (in minutes) to log data
const uint8_t  loggingInterval = 10;
// Your logger's timezone.
const int8_t timeZone = -6;  // Central Standard Time (CST=-6)
```

#### Device Addresses/Pins:

```C++
const char *apn = "apn.konekt.io";  // The APN for the gprs connection, unnecessary for WiFi
const int SonarData = 11;     // sonar data receive pin
byte y511modbusAddress = 0x03;  // The modbus address of the Y511 Turbidity
byte y520modbusAddress = 0x01;  // The modbus address of the Y520 Conductivity

```


#### Registration tokens:

```C++
const char *REGISTRATION_TOKEN = "d86bb6c7-e0c4-4c19-acb7-17a4c7f3eeca";   // Device registration token
const char *SAMPLING_FEATURE = "103b2e3e-2c53-4a39-8d76-3c2f0a016615";     // Sampling feature UUID
const char *UUIDs[] =                                                      // UUID array for device sensors
{
    "63d202e8-5294-4aa0-a4aa-5c57a62aa428",   // Free SRAM (EnviroDIY_Mayfly_FreeRAM)
    "e27eb08f-42ba-45e3-be52-5d946f7a5772",   // Temperature (Maxim_DS3231_Temp)
    "3a2de436-7468-4c5d-a88a-f079e5ad7e7d",   // Distance (MaxBotix_MB7389_Distance)
    "0e46b047-2675-4697-82f1-e9eab2facb74",   // Electrical conductivity (YosemiTech_Y520-A_Cond)
    "c7b8ca07-a214-47c9-8fe5-d47a601f49ee",   // Temperature (YosemiTech_Y520-A_Temp)
    "72b20e85-cc44-48a1-a384-d28901e38113",   // Turbidity (YosemiTech_Y511-A_Turbidity)
    "523bc735-ebca-47f2-a81e-38b1af23b658",   // Temperature (YosemiTech_Y511-A_Temp)
    "8f79d053-b990-4533-a6ee-513169803eec"    // Battery voltage (EnviroDIY_Mayfly_Batt)
};
```
