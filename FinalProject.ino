#include <TinyGPS.h>
#include <Wire.h>         // I2C communication  
#include "BMA250.h"       // Accelerometer library
#include <TinyScreen.h>   // TinyScreen library
#include "GPS.h"
#include <SPI.h>


// Accelerometer variables
BMA250 accel_sensor;
int x;
int y;
int z;
float accelMagnitude;
float SecondAccelMagnitude;

// Fall detection thresholds
float freeFallThreshold = 350;
#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#include <SoftwareSerial.h>
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#include "SoftwareSerialZero.h"
#endif
// Setup Serial for different boards
#if defined(ARDUINO_ARCH_SAMD)
 #define SerialMonitorInterface SerialUSB
#else
 #define SerialMonitorInterface Serial
#endif
TinyScreen display = TinyScreen(TinyScreenPlus);

const FONT_INFO& font10pt = thinPixel7_10ptFontInfo;
TinyGPS gps;
char buf[32];
SoftwareSerial softSerial(GPS_RXPin, GPS_TXPin);
#define Gps_serial softSerial

/* GPS related variables */
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long age;
float flat, flon;

/* Functions for serial monitor output */
static void smartdelay2(unsigned long ms);
static void print_float2(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void setup() {
    Wire.begin();

    /* TinyScreen setup */
    display.begin();
    display.on();
    display.setBrightness(10);

    delay(200);
    accel_sensor.begin(BMA250_range_2g, BMA250_update_time_64ms);

    // Initialize TinyScreen
    display.on();
    display.clearScreen();
    display.setFont(liberationSansNarrow_8ptFontInfo);
    display.fontColor(TS_8b_White,TS_8b_Black);
    display.setCursor(10,25);
    display.print("FALL DETECTION");
    delay(2000);
    display.clearScreen();
    /* Tinyscreen setup */
  Wire.begin();
  display.begin();
  display.on();
  display.setBrightness(10);

  /* Default display "GPS" */
  GPS_welcome();
  
  /* Serial monitor start */
  SerialMonitorInterface.begin(115200);
  while (!SerialMonitorInterface && millis() < 5000); //On TinyScreen+, this will wait until the Serial Monitor is opened or until 5 seconds has passed

  /* GPS setup */
  Gps_serial.begin(GPSBaud);
  gpsInitPins();
  delay(100);
  SerialMonitorInterface.print("Attempting to wake GPS module.. ");
  gpsOn();
  SerialMonitorInterface.println(" DONE.");
  delay(200);

  /* Enable and set interval or disable, per NMEA sentence type */
  Gps_serial.print(gpsConfig(NMEA_GGA_SENTENCE, 1));
  Gps_serial.print(gpsConfig(NMEA_GLL_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GSA_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GSV_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_RMC_SENTENCE, 1));
  Gps_serial.print(gpsConfig(NMEA_VTG_SENTENCE, 0));
  Gps_serial.print(gpsConfig(NMEA_GNS_SENTENCE, 0));

  /* read GPS software serial */
  while (Gps_serial.available()){
    Gps_serial.read();
  }
    
}

void loop() {
    
     
  smartdelay2(1000);
  
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;

  /* Serial monitor output */
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float2(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float2(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float2(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float2(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float2(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  SerialMonitorInterface.println();

  /* Display data to tinyscreen */
  display_coordinates();
  int timer = millis();
    String result = "Fall Confirmed!";

    while ((millis() - timer) < 15000){
        accel_sensor.read();  // Read accelerometer data
    x = accel_sensor.X;
    y = accel_sensor.Y;
    z = accel_sensor.Z;
    if (x == -1 && y == -1 && z == -1) {
        SerialMonitorInterface.println("ERROR");
        return;
    }

    accelMagnitude = sqrt(sq(x) + sq(y) + sq(z));  // Calculate magnitude'

    if (accelMagnitude > freeFallThreshold){
        delay(200);
        accel_sensor.read();
        int a = accel_sensor.X;
        int b = accel_sensor.Y;
        int c = accel_sensor.Z;
        SecondAccelMagnitude = sqrt(sq(a) + sq(b) + sq(c));
        if (SecondAccelMagnitude > freeFallThreshold){
            SerialMonitorInterface.println("Fall Detected");
            displayFallDetected();
        }
    }
    showSerial();  // Debugging output
    delay(250);
    }
  
  
}
static void smartdelay2(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Gps_serial.available()) {
      gps.encode(Gps_serial.read());
    }
  } while (millis() - start < ms);
}

static void print_float2(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      //SerialMonitorInterface.print('*');
      SerialMonitorInterface.print("");
    //SerialMonitorInterface.print(' ');
    SerialMonitorInterface.print("");
  }
  else
  {
    SerialMonitorInterface.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      SerialMonitorInterface.print(' ');
  }
  smartdelay2(0);
     // Adjust delay as necessary
}

void displayFallDetected() {
    display.clearScreen();
    display.setFont(font10pt);  // Choose a suitable font
    display.setCursor(10, 20);          // Set position (x, y)
    display.fontColor(TS_8b_White,TS_8b_Black);
    display.print("Fall Detected!");
    display.fontColor(TS_8b_Red,TS_8b_Black);
    display.setCursor(58, 10);
    display.print("Confirm");
    display.fontColor(TS_8b_Blue,TS_8b_Black);
    display.setCursor(60, 44);
    display.print("No Fall");
    display.fontColor(TS_8b_White,TS_8b_Black);

    int timer = millis();
    String result = "Fall Confirmed!";

    while ((millis() - timer) < 10000){
        float seconds = 10.0 - ((millis() - timer)/1000.0);
        // SerialMonitorInterface.println(seconds);
        display.setCursor(10, 30);
        display.fontColor(TS_8b_White,TS_8b_Black);
        display.print("Timer: ");
        display.setCursor(40, 30);
        display.print(seconds);
        delay(100);
        if (display.getButtons(TSButtonUpperRight)) {
            result = "Fall Confirmed!";
            break;
        }
        else if(display.getButtons(TSButtonLowerRight)){
            result = "False Alarm!";
            break;
        }
    }
    display.clearScreen();
    if ( result ==  "Fall Confirmed!"){
      display.fontColor(TS_8b_Red,TS_8b_Black);
      display.setCursor(13, 28);
      display.setFont(font10pt);
      display.print(result);
      SerialMonitorInterface.println("Notifying Caregiver...");
    }
    else if ( result == "False Alarm!"){
      display.setCursor(8, 20);
      display.fontColor(TS_8b_White,TS_8b_Black);
      display.setFont(liberationSans_12ptFontInfo);
      display.print(result);
      SerialMonitorInterface.println("False Alarm!");
    }
    delay(5000);
    display.clearScreen();      // Refresh the screen to show the message
    
}

void showSerial() {
    // SerialMonitorInterface.print("X = ");
    // SerialMonitorInterface.print(x);
    // SerialMonitorInterface.print("  Y = ");
    // SerialMonitorInterface.print(y);
    // SerialMonitorInterface.print("  Z = ");
    // SerialMonitorInterface.print(z);
    SerialMonitorInterface.print("  Accel Magnitude = ");
    SerialMonitorInterface.println(accelMagnitude);
}


static void print_float22(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      SerialMonitorInterface.print("");
      // SerialMonitorInterface.print('*');
    //SerialMonitorInterface.print(' ');
    SerialMonitorInterface.print("");
  }
  else
  {
    //SerialMonitorInterface.print(val, prec);
    SerialMonitorInterface.print("");
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      //SerialMonitorInterface.print(' ');
      SerialMonitorInterface.print("");
  }
  smartdelay2(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  //SerialMonitorInterface.print(sz);
  SerialMonitorInterface.print("");
  smartdelay2(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    SerialMonitorInterface.print("");
    //SerialMonitorInterface.print("********** ******** "); 
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    //SerialMonitorInterface.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay2(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    SerialMonitorInterface.print("");
    //SerialMonitorInterface.print(i<slen ? str[i] : ' ');
  smartdelay2(0);
}

/* Default display */
void GPS_welcome()
{
  display.on();
  display.clearScreen();
  display.setFont(liberationSansNarrow_12ptFontInfo);
  display.fontColor(TS_8b_Yellow,TS_8b_Black);
  display.setCursor(35,25);
  display.print("GPS");
  delay(2000);
  display.clearScreen();
  
}

/* Display GPS info on Tinyscreen */ 
static void display_coordinates()
{
    display.setFont(thinPixel7_10ptFontInfo);
    display.fontColor(TS_8b_Blue,TS_8b_Black);
    display.setCursor(0,0);
    display.print("[GPS data]");
    
    display.setFont(thinPixel7_10ptFontInfo);
    display.fontColor(TS_8b_White,TS_8b_Black);
    
    display.setCursor(0,15);
    display.print("Lat : ");
    if (flat == TinyGPS::GPS_INVALID_F_ANGLE){
      display.print("N/A");
      SerialMonitorInterface.println("N/A");
    } 
    else{
      display.print(flat);
      SerialMonitorInterface.println(flat);
    } 

    display.setCursor(0,30);
    display.print("Long: ");
    if (flon == TinyGPS::GPS_INVALID_F_ANGLE){
      display.print("N/A");
      SerialMonitorInterface.println("N/A");
    } 
    else{
      display.print(flon);
      SerialMonitorInterface.println(flon);
    } 
    
    display.setCursor(0,45);
    display.print("Head: ");
    if(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE){
      display.print("N/A");
      SerialMonitorInterface.println("N/A");
    } 
    else{
      display.println(TinyGPS::cardinal(gps.f_course()));
      SerialMonitorInterface.println("N/A");
    } 
}
