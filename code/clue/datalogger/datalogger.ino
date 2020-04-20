#include <Adafruit_Arcada.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h> //accel gyro temp
#include <Adafruit_LIS3MDL.h> // magnetos
#include <Adafruit_SHT31.h> // humidity
#include <Adafruit_BMP280.h> // temp baro
#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial gpsSerial(A6, A5); // RX, TX

#define LAP_INPUT_PIN 12
#define START_INPUT_PIN 10
#define STOP_INPUT_PIN 11

const unsigned long SAMPLE_DELAY = 20; //milliseconds
const uint8_t START_SENSOR_DATA = 0x40;
const int32_t SENSOR_TYPE_BUTTON = 20;

bool sentenceReady = false;
uint8_t lap = 0;
  
Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33; // accel gyro temp
Adafruit_LIS3MDL lis3mdl; // magneto
Adafruit_SHT31 sht30; // baro
Adafruit_BMP280 bmp280; // humidity
TinyGPS sgps;

bool sampling = false;
bool connected = false;
bool debug = false;
bool okay = true;
bool prevLap = false;

unsigned long prevMillis = 0;
int32_t lapStartTimeStamp = 0;
    
void setup() {

  if ( debug ) {
    Serial.begin(115200);
  }
  
  Serial1.begin(115200);
  gpsSerial.begin(4800);
      
  if ( debug ) {
    Serial.println("Telemetry Capturer");
  }
  
  if (!arcada.arcadaBegin()) {
    okay = false;
    if ( debug ) {
      Serial.print("Arcada failed to begin");
    }
  }
  
  arcada.displayBegin();

  if ( debug ) {
    Serial.println("starting capture kernel...");
  }

  arcada.setBacklight(240);
  arcada.display->setCursor(0, 0);
  arcada.display->setTextWrap(true);
  arcada.display->setTextSize(2);
  
  /********** Check filesystem next */
  if (!arcada.filesysBegin()) {
    okay = false;
    if ( debug ) {
      Serial.println("Failed to load filesys");
    }
  }
  
  /********** Check LSM6DS33 */
  if (!lsm6ds33.begin_I2C()) {
    okay = false;
    if ( debug ) {
      Serial.println("No LSM6DS33 found");
    }
  } 
  
  /********** Check LIS3MDL */
  if (!lis3mdl.begin_I2C()) {
    okay = false;
    if ( debug ) {
      Serial.println("No LIS3MDL found");
    }
  }
  
  /********** Check SHT3x */
  if (!sht30.begin(0x44)) {
    okay = false;
    if ( debug ) {
      Serial.println("No SHT30 found");
    }
  }

  /********** Check BMP280 */
  if (!bmp280.begin()) {
    okay = false;
    if ( debug ) {
      Serial.println("No BMP280 found");
    }
  }

  arcada.display->setTextWrap(true);

  pinMode( LAP_INPUT_PIN, INPUT_PULLUP ); 
  pinMode( START_INPUT_PIN, INPUT_PULLUP ); 
  pinMode( STOP_INPUT_PIN, INPUT_PULLUP ); 
  
  if ( okay ) {
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("Ready");
  } else {
    arcada.display->setTextColor(ARCADA_RED);
    arcada.display->println("Error");
  }
}

void WriteHeader() {
  Serial1.println("\r\nSample\n");
}

void WriteLap() {
  int32_t timestamp = millis();
    
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&SENSOR_TYPE_BUTTON, 4);
  Serial1.write((uint8_t*)&timestamp, 4);
  
  if ( digitalRead( LAP_INPUT_PIN ) == LOW && prevLap == false ) {
    prevLap = true;
    Serial1.write((uint8_t*)&lap, 1);
  } else if ( digitalRead( LAP_INPUT_PIN ) != LOW && prevLap == true ) {
    prevLap = false;   
    
    lap++;
    Serial1.write((uint8_t*)&lap, 1);
    
    int32_t lapDelta = timestamp - lapStartTimeStamp;
    const int numMinutes = lapDelta / 1000 / 60;
    const int numSeconds = lapDelta / 1000 % 60;
    const int numMillis = lapDelta % 1000;
    
    char laptext[16];
    memset(laptext,'\0', sizeof(laptext));
    snprintf(laptext, sizeof(laptext)-1, "Lap %d: %02d.%02d.%03d\n", lap, numMinutes, numSeconds, numMillis);
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->print( laptext );
    arcada.display->println(" ");

    lapStartTimeStamp = timestamp;
  } else {
    Serial1.write((uint8_t*)&lap, 1);
  }
  
  Serial1.write('\n');
}

void WriteAccel( const sensors_event_t & accel ) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&accel.type, 4);
  Serial1.write((uint8_t*)&accel.timestamp, 4);
  Serial1.write((uint8_t*)&accel.acceleration.v, 12);
  Serial1.write('\n');
}

void WriteGyro( const sensors_event_t & gyro ) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&gyro.type, 4);
  Serial1.write((uint8_t*)&gyro.timestamp, 4);
  Serial1.write((uint8_t*)&gyro.gyro.v, 12);
  Serial1.write('\n');
}

void WriteMag( const sensors_event_t & mag ) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&mag.type, 4);
  Serial1.write((uint8_t*)&mag.timestamp, 4);
  Serial1.write((uint8_t*)&mag.magnetic.v, 12);
  Serial1.write('\n');
}

void WriteTemp(const sensors_event_t & temp) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&temp.type, 4);
  Serial1.write((uint8_t*)&temp.timestamp, 4);
  Serial1.write((uint8_t*)&temp.temperature, 4);
  Serial1.write('\n');
}

void WriteBaro(const sensors_event_t & pres) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&pres.type, 4);
  Serial1.write((uint8_t*)&pres.timestamp, 4);
  Serial1.write((uint8_t*)&pres.pressure, 4);
  Serial1.write('\n');
}

void WriteHumid(const sensors_event_t & humid) {
  Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
  Serial1.write((uint8_t*)&humid.type, 4);
  Serial1.write((uint8_t*)&humid.timestamp, 4);
  Serial1.write((uint8_t*)&humid.relative_humidity, 4);
  Serial1.write('\n');
}

void parseGPS() {
  while ( gpsSerial.available() ) {
    if ( sgps.encode( gpsSerial.read() ) ) {
      sentenceReady = true;
      break;
    }
  }
}

void WriteGPS() {
  if ( sentenceReady ) {
    sensors_gps_t gps;
    sgps.f_get_position(&gps.latitude, &gps.longitude, &gps.age);
    gps.status = gps.latitude != TinyGPS::GPS_INVALID_F_ANGLE && gps.longitude != TinyGPS::GPS_INVALID_F_ANGLE && gps.age != TinyGPS::GPS_INVALID_AGE;
    
    int32_t timeStamp = millis();
    int32_t gpstype = SENSOR_TYPE_GPS;
    Serial1.write((uint8_t*)&START_SENSOR_DATA, 1);
    Serial1.write((uint8_t*)&gpstype, 4);
    Serial1.write((uint8_t*)&timeStamp, 4);
    Serial1.write((uint8_t*)&gps.latitude, 4);
    Serial1.write((uint8_t*)&gps.longitude, 4);
    Serial1.write((uint8_t*)&gps.age, 4);
    Serial1.write((uint8_t*)&gps.status, 1);
    Serial1.write('\n');
    sentenceReady = false;
  }
}

void getHumidityEvent( sensors_event_t & humidity ) {
  humidity.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  humidity.timestamp = millis();
  humidity.relative_humidity = sht30.readHumidity();
}

void loop() {

  if ( okay == false ) {
    return;
  }

  if ( connected == false ) {
    if ( Serial1.available() ) {      
      String temp = Serial1.readString();
      if ( temp.startsWith("ready") ) {
        connected = true;
        tone(ARCADA_AUDIO_OUT, 6000, 200);
        arcada.display->setTextColor(ARCADA_BLUE);
        arcada.display->println("Connected!");
      }
    }
    return;
  }

  uint8_t pressed_buttons = arcada.readButtons();
  if (digitalRead( START_INPUT_PIN ) == LOW && sampling == false ) {
      tone(ARCADA_AUDIO_OUT, 4000, 100);
      arcada.display->println("Starting Capture");
      arcada.display->setTextColor(ARCADA_YELLOW);
      arcada.display->println("Sampling...");
      sampling = true;
      lap = 0;
      lapStartTimeStamp = prevMillis = millis();
  }
  
  if ( digitalRead( STOP_INPUT_PIN ) == LOW && sampling == true ) {
      sampling = false;
      tone(ARCADA_AUDIO_OUT, 5000, 100);
      arcada.display->setTextColor(ARCADA_BLUE);
      arcada.display->println("Stopped Capture");
      arcada.display->setTextColor(ARCADA_GREEN);
      arcada.display->println("Ready");
  }

  unsigned long nextSampleTime = prevMillis + SAMPLE_DELAY;
  unsigned long currentSampleTime = millis();
  if ( sampling == true && currentSampleTime > nextSampleTime ) {
    prevMillis = currentSampleTime;
    sensors_event_t accel, gyro, mag, temp, pres, humid;
    lsm6ds33.getEvent(&accel, &gyro, 0);
    lis3mdl.getEvent(&mag);
    bmp280.getTemperatureSensor()->getEvent(&temp);
    bmp280.getPressureSensor()->getEvent(&pres);
    getHumidityEvent( humid ); 
    parseGPS();
    
    WriteHeader();      
    WriteLap();
    WriteTemp( temp );
    WriteAccel( accel );
    WriteGyro( gyro );
    WriteMag( mag );
    WriteBaro( pres );
    WriteHumid( humid );
    WriteGPS();
  } 
}
