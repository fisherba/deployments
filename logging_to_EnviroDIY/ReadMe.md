# Example using the Modular Sensors Library to Save Data to an SD card and Send data to the EnviroDIY Data Portal

This shows the use of an "EnviroDIY logger" object for an AVR board.  It takes the simple_logging example one step farther in creating a modem object that is used to send data to the EnviroDIY data portal.  Before using this example, you must register a site and sensors at the data portal (http://data.envirodiy.org/).  After you have registered the site and sensors, the portal will generate a registration token and universally unique identifier (UUID) for each site and further UUID's for each variable.  You will need to copy all of those UUID values into your sketch to replace the "12345678-abcd-1234-efgh-1234567890ab" place holders in this example.  Please do not try to run the example exactly as written, but delete the chunks of code pertaining to sensors that you do not have attached.


const char *REGISTRATION_TOKEN = "fa5d2105-6b3c-4cd4-8715-71c5ae3e6e7a";   // Device registration token
const char *SAMPLING_FEATURE = "a8953b9e-bd86-4ec6-93f6-c5ba54e3c991";     // Sampling feature UUID
const char *UUIDs[] =                                                      // UUID array for device sensors
{
    "05458ff1-0f46-4cc1-a04a-a0e2d208064d",   // Battery voltage (EnviroDIY_Mayfly_Batt)
    "5efa56c9-6604-4606-bb56-140c0ee73416",   // Altitude (Bosch_BME280_Altitude)
    "e16ae034-1b5b-4fbe-87d1-4875b4ba07cf",   // Barometric pressure (Bosch_BME280_Pressure)
    "73615068-83a4-42dd-99cd-9368986d7fad"    // Temperature (Maxim_DS3231_Temp)
};
