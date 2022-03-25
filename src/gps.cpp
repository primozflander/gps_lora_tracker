#include "Gps.h"

// HardwareSerial GPSSerial(1);

void GPS::init()
{
//   GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial1.setTimeout(2);
}

void GPS::encode()
{       
    int data;
    int previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (Serial1.available() )
        {
            char data = Serial1.read();
            // Serial.print(data);
            // tGps.encode(*gpsStream++);
            tGps.encode(data);
        }
    }
}


void GPS::encodeDebug()
{       
    int data;
    int previousMillis = millis();

    while((previousMillis + 1000) > millis())
    {
        while (Serial1.available() )
        {
            // char data = Serial1.read();
            // Serial.print(data);
            tGps.encode(*gpsStream++);
        }
    }
}


void GPS::getLatLon(double* lat, double* lon, double *alt, double *kmph, int *sats)
{
  sprintf(t, "Lat: %f", tGps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", tGps.location.lng());
  Serial.println(t);
  
  sprintf(t, "Alt: %f meters", tGps.altitude.meters());
  Serial.println(t);

  sprintf(t, "Speed: %f km/h", tGps.speed.kmph());
  Serial.println(t);

  sprintf(t, "Sats: %d", tGps.satellites.value());
  Serial.println(t);

  *lat = tGps.location.lat();
  *lon = tGps.location.lng();
  *alt = tGps.altitude.meters();
  *kmph = tGps.speed.kmph();
  *sats = tGps.satellites.value();
}

bool GPS::checkGpsFix()
{
  encode();
  if (tGps.location.isValid() && 
      tGps.location.age() < 2000 &&
      tGps.hdop.isValid() &&
      tGps.hdop.value() <= 300 &&
      tGps.hdop.age() < 2000 &&
      tGps.altitude.isValid() && 
      tGps.altitude.age() < 2000 )
  {
    Serial.println("Valid gps Fix.");
    return true;
  }
  else
  {
    Serial.println("No gps Fix.");
    return false;
  }
}
