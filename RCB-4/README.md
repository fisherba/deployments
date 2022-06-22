# Deployment Notes

#### SiteCode: RCB-4
https://monitormywatershed.org/sites/RCB-4/


#### Library Dependencies (in PlatformIO.ini)

```.ini
build_flags =
    -DSDI12_EXTERNAL_PCINT
    -DNEOSWSERIAL_EXTERNAL_PCINT
    -DMQTT_MAX_PACKET_SIZE=240
    -DTINY_GSM_RX_BUFFER=64
    -DTINY_GSM_YIELD_MS=2
    -DENABLE_SERIAL2
    -DENABLE_SERIAL3
    -D BUILD_MODEM_XBEE_CELLULAR  ; Turn on first time w/ a Digi LTE-M module
lib_deps =
    EnviroDIY_ModularSensors@=0.32.2
    https://github.com/PaulStoffregen/AltSoftSerial.git
    https://github.com/SRGDamia1/NeoSWSerial.git
    https://github.com/EnviroDIY/SoftwareSerial_ExternalInts.git
```

#### Logger Settings

```C++
```

#### Device Addresses/Pins:

```C++
```


#### Registration tokens:

```C++
const char *UUIDs[] =                                                      // UUID array for device sensors
{
    "63804d8a-ebf8-42d4-a102-e2f290df1e75",   // Gage height (All_Calc_gageHeight)
    "339846da-f5b8-47e6-b11d-52f950d64f27",   // Battery voltage (EnviroDIY_Mayfly_Batt)
    "7fc331b0-8ad4-411f-89b2-ad20fa65db36",   // Turbidity (YosemiTech_Y511-A_Turbidity)
    "ad152474-024d-46d7-8c00-a317a1bf4f45",   // Temperature (YosemiTech_Y511-A_Temp)
    "a695d138-8db3-4715-9479-e3a3a1e5e437",   // Electrical conductivity (YosemiTech_Y520-A_Cond)
    "1fda2905-dd5c-4ddd-b677-a9372b598a6d",   // Temperature (YosemiTech_Y520-A_Temp)
    "feabb10d-3abc-4753-923b-76ae3399066d",   // Percent full scale (Digi_Cellular_SignalPercent)
    "e98c05c1-59f4-4e59-bef5-d6c5b40b2084"    // Sequence number (EnviroDIY_Mayfly_SampleNum)
};
const char *registrationToken = "1cfdc50c-2570-49f2-a7a9-8d4cb8b68a86";   // Device registration token
const char *samplingFeature = "e0a3a326-5edf-4bfc-be6c-cbd120ee390c";     // Sampling feature UUID
```
