#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_NeoPixel.h>

#define GPS_ECHO_ENABLED  false

#define NEOPIXEL_PIN      6
#define PIXEL_COUNT       24
#define PIXEL_COLOUR      0xFF00FF

// Calibration offsets
#define HEADING_OFFSET    0
#define LED_RING_OFFSET   -5

#define NUM_TIMERS        2
#define UPDATE_INTERVAL   100
#define OUTPUT_INTERVAL   2000

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_NeoPixel strip(PIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct coord
{
  float lat;
  float lon;
};

uint32_t timers[NUM_TIMERS];

void setup()
{
  // Initialise pixel ring
  strip.begin();              // Initialise pixels
  strip.show();               // Ensure all pixels are clear
  strip.setBrightness(50);    // Set pixel brightness going forward
  
  Serial.begin(115200);
  Serial.println("GPS Satellite navigator, initialising...");

  // Initialise GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(2000);

  // Initialise magnetometer
  mag.enableAutoRange(true);
  if(!mag.begin())
  {
    Serial.println("Failed to detect magnetometer, halting...");
    while(1);
  }

  delay(5000);

  // Initialise timers
  for(int i = 0; i < NUM_TIMERS; i++)
  {
    timers[i] = millis();
  }
}

void loop()
{
  static const float dest_lat = 50.424061;
  static const float dest_lon = -3.596950;

  /* GPS Section */
  float cur_dist_to_travel = -1;
  float cur_bearing = -1;
  
  coord current_pos = get_current_position();
  if(current_pos.lat != 0.0 && current_pos.lon != 0.0)
  {
    cur_dist_to_travel = calculate_distance(current_pos.lat, current_pos.lon, dest_lat, dest_lon);
    cur_bearing = calculate_bearing(current_pos.lat, current_pos.lon, dest_lat, dest_lon);
  }
  /* End of GPS Section */

  // If timers wrap around, just reset them
  for(int i = 0; i < NUM_TIMERS; i++)
  {
    if(timers[i] > millis())
    {
      timers[i] = millis();
    }
  }

  // Poll sensors (using 1st timer)
  float compass_heading;
  float led_ring_heading;
  if((millis() - timers[0]) > UPDATE_INTERVAL)
  {
    timers[0] = millis(); // reset the timer

    /* Compass Section */
    static int previous_heading = -1;
    compass_heading = get_compass_heading();
    if(compass_heading != previous_heading)
    {
      previous_heading = compass_heading;

      led_ring_heading = cur_bearing - compass_heading;
      // Normalise bearing (0-360)
      if(led_ring_heading < 0)
      {
        led_ring_heading += 360;
      }
      else if(led_ring_heading > 360)
      {
        led_ring_heading -= 360;
      }

      // Map LED ring heading range to LED index range
      int led_index = (int)round(map(led_ring_heading, 0, 360, 0, PIXEL_COUNT - 1));
  
      // Apply LED offset
      led_index += LED_RING_OFFSET;
      // Wrap around LED offset if out of bounds
      if(led_index < 0)
      {
        led_index += PIXEL_COUNT;
      }
      else if(led_index > (PIXEL_COUNT - 1))
      {
        led_index -= PIXEL_COUNT;
      }
  
      // Update LED ring display
      strip.clear();
      strip.setPixelColor(led_index, PIXEL_COLOUR);
      strip.show();
    }
    /* End of Compass Section */
  }

  // Output stats to connected PC (2nd timer)
  if((millis() - timers[1]) > OUTPUT_INTERVAL)
  {
    timers[1] = millis(); // reset the timer
    
    /* Serial Output Section */
    Serial.println("");
    if(GPS.fix)
    {
      Serial.print("Current lat/long: ");
      Serial.print(current_pos.lat, DEC);
      Serial.print(", ");
      Serial.println(current_pos.lon, DEC);
  
      Serial.print("Distance to destination (metres): ");
      Serial.println(cur_dist_to_travel);

      Serial.print("Heading between current and destination coords: ");
      Serial.println(cur_bearing);

      Serial.print("LED ring heading: ");
      Serial.println(led_ring_heading);
    }
    else
    {
      Serial.println("No fix :(");
    }
    /* End of Output Section */
  }
}

// Gets current position as decimal lat/long coordinate
coord get_current_position()
{
  coord current_pos = {0.0, 0.0};
  
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPS_ECHO_ENABLED))
  {
    Serial.write(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    if(!GPS.parse(GPS.lastNMEA()))
    {
      return current_pos;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  
  // Calculate decimal values for lat/long, and return
  current_pos.lat = decimal_degrees(GPS.latitude, GPS.lat);
  current_pos.lon = decimal_degrees(GPS.longitude, GPS.lon);

  return current_pos;
}

// Gets current heading in degrees from magnetometer
float get_compass_heading()
{
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the angle of the vector x,y
  float cur_heading = ((float)atan2(event.magnetic.y, event.magnetic.x) * 180) / PI;
  // Invert heading (seems to be required)
  //cur_heading = (360 - cur_heading);

  // Normalise heading to 0-360
  if(cur_heading < 0)
  {
    cur_heading += 360;
  }

  // Apply heading calibration offset
  cur_heading += HEADING_OFFSET;
  if(cur_heading < 0)
  {
    cur_heading += 360;
  }
  else if(cur_heading > 360)
  {
    cur_heading -= 360;
  }

  return cur_heading;
}

// Convert NMEA coordinate to decimal degrees
float decimal_degrees(float nmea_coord, char dir)
{
  uint16_t wholeDegrees = 0.01*nmea_coord;
  int modifier = 1;
 
  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }
  
  return (wholeDegrees + (nmea_coord - 100.0*wholeDegrees)/60.0) * modifier;
}

// Calculate distance in metres between two lat/long coordinates
unsigned long calculate_distance(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
 
  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));
 
  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;
 
  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
 
  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

int calculate_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;
 
  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
  calc=atan2(y,x);
 
  bear_calc= degrees(calc);
 
  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}
