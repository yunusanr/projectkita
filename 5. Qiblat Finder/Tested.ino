#include <TinyGPS++.h>       // Library for GPS
#include <SoftwareSerial.h>  // Library for software serial communication
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include <EEPROMex.h>
#include <LiquidCrystal_I2C.h>
#include <MechaQMC5883.h>

LiquidCrystal_I2C s_lcd(0x27, 16, 2);
LiquidCrystal_I2C b_lcd(0x26, 20, 4);

MechaQMC5883 qmc;

int x1, y1, z1;
float heading = 0.0;

// Number of decimal digits to print
const uint8_t digits = 3;

int aaa;
int Hour;
int Minute;
int Second;
int Day;
int Month;
int Year;

float lintang_K = 21.42250833;  // Latitude of Kaaba
float bujur_K = 39.82616111;    // Longitude of Kaaba
int Zona = 7;                   // Time zone

float lintang_T;
float bujur_T;

// Details of Calculation
float KIB;
float NNN = 294.7;
float CIB;
float Arah_Kiblat;

// Helper Variables
float pembilang;
float penyebut;
float p1, p2, p3, p4, x;

// Details of Calculation
int M;
int Y;
int A;
int B;

// Helper Variables
int b2;

float JD, JDE, T, JD_0_UT;
float T_U_JD;

// Helper Variables
long v2;
int v3;
float v4;
float v5;

// Calculating Delta T
float Delta_T;
float Delta_Tahun;
float Tahun;

// Position of Sun Variables
float L0, atas, M0, atas2, C, e, Ma, L, atas8;
float Omega, atas3, atas4, atas5, atas6, atas7, Epsilon_0, Delta_Epsilon, Epsilon;
float GST_0_UT, GST_Lokal, LST, Lambda, Alpha, Delta, HA;
float Azimuth, Azimuth2, Azimuth_Bayangan;

// Choose two Arduino pins to use for software serial
int RXPin = 6;  // Connect to TX GPS
int TXPin = 7;  // Connect to RX GPS

int GPSBaud = 9600;  // Default baud rate

// Creating TinyGPS++ object
TinyGPSPlus gps;

float CC;
float Mundur_Shof;
float tigadua;

int relay = 4;
int red = 12;

int counter = 0;
int attempts = 0;
int max_attempts = 3;

String mymints;
float minutes = 0;
String mytiga;
float tiga = 0;

String mysecs;
float seconds = 0;
float total_seconds = 0;
int secflag = 0;
int secflag2 = 0;
int timer_started_flag = 0;

// Tracks the time since last event fired
unsigned long previousMillis = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs = 0;
unsigned long currentMillis = 0;
int interval = 1000;  // Updated every 1 second
int tsecs = 0;

float last_kib, last_bay, last_sec;
bool first_rotate = false;
float compass_azimuth;

// Making connection serial with name "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
  // Starting serial connection at baud rate 9600
  Serial.begin(9600);

  // Starting serial connection with GPS sensor
  gpsSerial.begin(GPSBaud);

  // Pin setup
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

  // init_eeprom();
  init_lcd();
  // init_keypad();
  init_compass();
  init_actuator();
}

void loop() {
  // Display data when there is a connection
  while (gpsSerial.available() > 0) {
    update_compass();
    if (gps.encode(gpsSerial.read())) {
      displayInfo();
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;  // Save the last time the LCD was updated
        refresh_lcd();
        Serial.print("Heading: ");
        Serial.println(heading);
      }

      if (gps.location.isValid()) {
        refresh_lcd();
        delay(100);
        rotate_by_compass();
      }

      if (heading >= KIB - 2 && heading <= KIB + 2) {
        digitalWrite(relay, HIGH);
      }

      last_kib = KIB;
      last_bay = Azimuth_Bayangan;
    }
  }

  // If no connection in 5 seconds, display error "No GPS detected"
  // Check connection and reset Arduino
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
    while (true)
      ;
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    Serial.println("==========================================================");

    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);

    // Time calculation
    aaa = ((gps.time.hour()) + Zona);
    if (aaa < 24) {
      Hour = aaa;
    } else if (aaa > 23) {
      Hour = aaa - 24;
    }
    Serial.print("nilai x = ");
    Serial.println(Hour);

    Serial.print("DATE: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
    Serial.print("TIME: ");
    Serial.print(Hour);
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    Serial.print("LINTANG: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("BUJUR: ");
    Serial.println(gps.location.lng(), 6);

    // Qibla Direction Calculation
    lintang_T = (gps.location.lat());
    bujur_T = (gps.location.lng());
    CIB = bujur_K - bujur_T;
    pembilang = sin(CIB * DEG_TO_RAD);
    p1 = cos(lintang_T * DEG_TO_RAD);
    p2 = tan(lintang_K * DEG_TO_RAD);
    p3 = sin(lintang_T * DEG_TO_RAD);
    p4 = cos(CIB * DEG_TO_RAD);
    penyebut = (p1 * p2) - (p3 * p4);
    x = pembilang / penyebut;
    KIB = (atan2(pembilang, penyebut) * RAD_TO_DEG);
    if (KIB < 0) {
      KIB = KIB + 360;
    } else if (KIB > 0) {
      KIB = KIB;
    }
    Serial.print("Az.Kiblat: ");
    Serial.println(KIB, 1);

    // Sun Azimuth Calculation
    Minute = (gps.time.minute());
    Second = (gps.time.second());
    Day = (gps.date.day());

    // Month calculation
    if ((gps.date.month()) < 3) {
      Month = (gps.date.month()) + 12;
    } else if ((gps.date.month()) > 2) {
      Month = (gps.date.month());
    }

    // Year calculation
    if ((gps.date.month()) < 3) {
      Year = (gps.date.year()) - 1;
    } else if ((gps.date.month()) > 2) {
      Year = (gps.date.year());
    }

    // A calculation
    A = Year / 100;

    // B calculation
    b2 = A / 4;
    B = 2 + b2 - A;

    // JD calculation
    v2 = 365.25 * Year;
    v3 = 30.6001 * (Month + 1);
    v4 = ((Hour) + (float(Minute) / 60) + (float(Second) / 3600)) / 24;
    v5 = float(Zona) / 24;
    JD = 1720994.5 + v2 + v3 + B + Day + v4 - v5;

    // Delta T calculation (for years 2005-2150)
    Tahun = float(Year) + ((float(Month) - 1) / 12) + ((float(Day) / 365));
    if ((Tahun <= 2050) && (Tahun > 2005)) {
      Delta_Tahun = 62.92 + (0.32217 * (Tahun - 2000)) + (0.005589 * (Tahun - 2000) * (Tahun - 2000));
    } else if ((Tahun > 2050) && (Tahun <= 2150)) {
      Delta_Tahun = -20 + (32 * ((Tahun - 1820) / 100) * ((Tahun - 1820) / 100)) - (0.5628 * (2150 - Tahun));
    }
    Delta_T = Delta_Tahun / 86400;

    // JDE and T calculation
    JDE = JD + Delta_T;
    T = (JDE - 2451545) / 36525;

    // Mean Longitude (L0) calculation
    atas = (280.46646) + (36000.76983 * T) + (0.0003032 * T * T);
    L0 = fmod(atas, 360);

    // Mean Anomaly (M0) calculation
    atas2 = ((357.52911) + (35999.05029 * T) - (0.0001537 * T * T));
    M0 = fmod(atas2, 360);

    // Correction (C) calculation
    C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * sin((M0)*DEG_TO_RAD) + (0.019993 - 0.000101 * T) * sin(2 * (M0)*DEG_TO_RAD) + 0.000289 * sin(3 * (M0)*DEG_TO_RAD);

    // Eccentricity of Earth's Orbit (e) calculation
    e = (0.016708634) - (0.000042037 * T) - (0.0000001267 * T * T);

    // True Longitude (L) and True Anomaly (M) calculation
    L = L0 + C;
    Ma = M0 + C;

    // Omega calculation
    atas3 = (125.04) - (1934.136 * T);
    Omega = fmod(atas3, 360);
    if (Omega < 0) {
      Omega = Omega + 360;
    } else {
      Omega = Omega;
    }

    // Epsilon Zero calculation
    Epsilon_0 = (23) + (float(26) / 60) + (21.448 / 3600) - (46.815 * T / 3600) - (0.00059 * T * T / 3600) + (0.001813 * T * T * T / 3600);

    // Delta Epsilon and Epsilon calculation
    Delta_Epsilon = 0.00256 * cos(Omega * DEG_TO_RAD);
    Epsilon = Epsilon_0 + Delta_Epsilon;

    // JD at 0 UT calculation
    JD_0_UT = 1720994.5 + long(365.25 * Y) + int(30.60001 * (M + 1)) + B + Day;

    // T for JD calculation
    T_U_JD = (JD_0_UT - 2451545) / 36525;

    // GST at 0 UT calculation
    atas4 = (6.6973745583) + (2400.0513369072 * T_U_JD) + (0.0000258622 * T_U_JD * T_U_JD);
    GST_0_UT = fmod(atas4, 24);

    // GST Local calculation
    atas5 = (GST_0_UT) + ((float(Hour)) + (float(Minute) / 60) + (float(Second) / 3600 - Zona)) * (1.00273790935);
    GST_Lokal = fmod(atas5, 24);

    // LST calculation
    atas6 = (GST_Lokal) + (float(bujur_T) / 15);
    LST = fmod(atas6, 24);

    // Apparent Longitude (Lambda) calculation
    Lambda = L - 0.00569 - 0.00478 * sin(Omega * DEG_TO_RAD);

    // Alpha calculation
    atas7 = (atan2((cos(Epsilon * DEG_TO_RAD) * sin(Lambda * DEG_TO_RAD)), cos(Lambda * DEG_TO_RAD)) * RAD_TO_DEG);
    Alpha = fmod(atas7, 360);

    // Delta calculation
    Delta = (asin(((sin(Epsilon * DEG_TO_RAD)) * (sin(Lambda * DEG_TO_RAD)))) * RAD_TO_DEG);

    // Hour Angle (HA) calculation
    HA = (fmod((LST - (Alpha / 15)), 24)) * 15;

    // Azimuth (A) calculation
    atas8 = (cos(HA * DEG_TO_RAD) * sin(lintang_T * DEG_TO_RAD)) - (tan(Delta * DEG_TO_RAD) * cos(lintang_T * DEG_TO_RAD));
    Azimuth = atan2(sin(HA * DEG_TO_RAD), atas8) * RAD_TO_DEG;
    Azimuth2 = fmod(Azimuth + 180, 360);
    if (Azimuth2 < 180) {
      Azimuth_Bayangan = Azimuth2 + 180;
    } else {
      Azimuth_Bayangan = Azimuth2 - 180;
    }
    Serial.print("Az.Bay.Mat: ");
    Serial.println(Azimuth_Bayangan, 1);

    Serial.print("Arah Kiblat: ");
    Serial.println(KIB);
    Serial.println("==========================================================");

    // delay(1000);
  }
}

void refresh_lcd() {
  b_lcd.clear();
  s_lcd.clear();
  display_lcd();  // Refresh the LCD display
}

void init_lcd() {
  s_lcd.init();
  b_lcd.init();
  s_lcd.backlight();
  b_lcd.backlight();

  s_lcd.setCursor(4, 0);
  s_lcd.print("AZIMUTH");
  s_lcd.setCursor(2, 1);
  s_lcd.print("INFORMATION");

  b_lcd.setCursor(8, 0);
  b_lcd.print("LOCS");
  b_lcd.setCursor(9, 1);
  b_lcd.print("&&");
  b_lcd.setCursor(8, 2);
  b_lcd.print("TIME");
  b_lcd.setCursor(4, 3);
  b_lcd.print("INFORMATION");

  delay(5000);
  s_lcd.clear();
  b_lcd.clear();
}

void display_lcd() {
  s_lcd.setCursor(0, 0);
  s_lcd.print("Az.Kiblat: ");
  s_lcd.print(KIB, 1);
  s_lcd.setCursor(0, 1);
  s_lcd.print("Heading:");
  s_lcd.print(heading, 1);

  b_lcd.setCursor(0, 0);
  b_lcd.print("DATE: ");
  b_lcd.print(gps.date.day());
  b_lcd.print(" / ");
  b_lcd.print(gps.date.month());
  b_lcd.print(" / ");
  b_lcd.print(gps.date.year());

  b_lcd.setCursor(0, 1);
  b_lcd.print("TIME: ");
  b_lcd.print(Hour);
  b_lcd.print(" : ");
  b_lcd.print(gps.time.minute());
  b_lcd.print(" : ");
  b_lcd.print(gps.time.second());

  // if (KIB != last_kib || Azimuth_Bayangan != last_bay) s_lcd.clear();

  b_lcd.setCursor(0, 2);
  b_lcd.print("LAT : ");
  b_lcd.print(lintang_T, 6);
  b_lcd.setCursor(0, 3);
  b_lcd.print("LON : ");
  b_lcd.print(bujur_T, 6);

  last_sec = gps.time.second();
}
