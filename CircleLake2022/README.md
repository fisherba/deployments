# Deployment Notes

#### SiteCode: RPB-4
Site Link: 



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
const char *sketchName = "name.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "name_Mayfly_170289";
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


```
