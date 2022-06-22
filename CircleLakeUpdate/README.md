# Deployment Notes

#### SiteCode: RPB-4
Site Link: 



#### Library Dependencies (in PlatformIO.ini)

```.ini
[platformio]
description = beth deployments using ModularSensors
src_dir = CircleLakeUpdate
;src_dir = simple_logging
;src_dir = Example_04_Mayfly_setRTC

[env:mayfly]
board = mayfly
platform = atmelavr
framework = arduino
monitor_speed = 115200
lib_ldf_mode = deep+
lib_ignore = 
	RTCZero
	Adafruit NeoPixel
	Adafruit GFX Library
	Adafruit SSD1306
	Adafruit ADXL343
	Adafruit STMPE610
	Adafruit TouchScreen
	Adafruit ILI9341
build_flags = 
	-D SDI12_EXTERNAL_PCINT
	-D NEOSWSERIAL_EXTERNAL_PCINT
	-D TINY_GSM_RX_BUFFER=64
	-D TINY_GSM_YIELD_MS=2
	-D MQTT_MAX_PACKET_SIZE=240
	-D BUILD_MODEM_XBEE_CELLULAR
	-D MS_RAIN_SOFTWAREWIRE
lib_deps = 
	EnviroDIY_ModularSensors@=0.32.2
	https://github.com/PaulStoffregen/AltSoftSerial.git
	https://github.com/SRGDamia1/NeoSWSerial.git
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
