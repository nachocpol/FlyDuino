/*
 * How To Use:
 * First, run with k_resetMemory = true, this sets the mem banks to 0
 * Then compile and upload with it set to false.
 * Use it! come back to your PC and connect it to the serial monitor. 
 * Profit.
 */

//////////////INCLUDES///////////////

#include "Seeed_BME280.h"
#include <Wire.h>
#include <EEPROM.h>

//////////////GLOBALS///////////////

BME280 bme280;
const float k_floatMin = -3.4028235E+38;
const float k_floatMax = 3.4028235E+38;
const bool k_resetMemory = false;
const unsigned int k_floatSize = sizeof(float);
float g_maxAltitude = k_floatMin;
float g_initialAltitude = 0.0f;
float g_trackedMaxAltitude = 0.0f;
unsigned long g_elapsedForSave = 0; // In ms

//////////////FUNCTIONS/////////////

float QueryAltitude()
{
  float pressure = bme280.getPressure();  // In pascals
  return bme280.calcAltitude(pressure);   // In meters
}

void setup()
{
  Serial.begin(9600);
  
  // Init the BME, lets wait 4secs and then take the base altitude:
  if(!bme280.init())
  {
    Serial.println("Device error!");
  }
  delay(6000);
  g_initialAltitude = QueryAltitude();
  Serial.print("Initial altitude = ");Serial.println(g_initialAltitude);

  // Ensure correct state
  for(unsigned int i = 0; i < k_floatSize; ++i)
  {
    byte cur = EEPROM.read(i);
    if(cur == 0xFF || k_resetMemory) // 255 is the factory value
    {
      EEPROM.write(i,0x0);
      Serial.println("Triggered mem reset!");
    }
  }

  // Track
  EEPROM.get(0x0, g_trackedMaxAltitude);
  Serial.print("Tracked max altitude = "); Serial.println(g_trackedMaxAltitude);
}

void loop()
{
#if SHOW_DATA
  return;
#endif
  float sample = QueryAltitude();
  float curAltitude = sample - g_initialAltitude;
  if((curAltitude > g_maxAltitude) && (curAltitude > g_trackedMaxAltitude))
  {
    g_maxAltitude = curAltitude;
    Serial.println(g_maxAltitude);
  }

  // EEPROM has a limited ammount of writes, doing this
  // we prevent doing non essential writes
  g_elapsedForSave += 250; // uhm..
  if(g_elapsedForSave >= (1000 * 15)) // 15 seconds
  {
    g_elapsedForSave = 0;
    EEPROM.put(0x0, g_maxAltitude);
    Serial.println("Updating memory.");
  }
  
  delay(250);
}

/////////////////////////////////////////
