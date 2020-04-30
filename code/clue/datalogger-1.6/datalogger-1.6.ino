#include <Adafruit_Arcada.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h> //accel gyro temp
#include <Adafruit_LIS3MDL.h> // magnetos
#include <Adafruit_SHT31.h> // humidity
#include <Adafruit_BMP280.h> // temp baro
#include <SparkFun_Ublox_Arduino_Library.h> //gps
#include <bluefruit.h>

#define START_INPUT_PIN 10
#define STOP_INPUT_PIN 11

#define UPDATE_RATE 20
const unsigned long SAMPLE_DELAY = 1000 / 20; //milliseconds

Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33; // accel gyro temp
Adafruit_LIS3MDL lis3mdl; // magneto
Adafruit_SHT31 sht30; // baro
Adafruit_BMP280 bmp280; // humidity
SFE_UBLOX_GPS gps; //gps

BLEService mainService = BLEService(0x00000001000000fd8933990d6f411ff8);
BLECharacteristic gpsMainCharacteristic = BLECharacteristic (0x03);
BLECharacteristic gpsTimeCharacteristic = BLECharacteristic (0x04);

bool sampling = false;
bool connected = false;
bool okay = true;
bool debug = true;

unsigned long prevMillis = 0;
int gpsPreviousDateAndHour = 0;
uint8_t gpsSyncBits = 0;
uint8_t tempData[20];

void setup() {

  Wire.begin();
  Serial.begin(115200);
  while ( !Serial ) delay(10);

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("QuinlanSoftwareIMU");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  mainService.begin();
  gpsMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  gpsMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gpsMainCharacteristic.begin();
  gpsTimeCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  gpsTimeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  gpsTimeCharacteristic.begin();


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

  /******** check gps */
  if (gps.begin(Wire, 0xA1) == false)
  {
    okay = false;
    if ( debug ) {
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    }
  }

  arcada.display->setTextWrap(true);

  pinMode( START_INPUT_PIN, INPUT_PULLUP );
  pinMode( STOP_INPUT_PIN, INPUT_PULLUP );

  startAdv();

  if ( okay ) {
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("Ready");
  } else {
    arcada.display->setTextColor(ARCADA_RED);
    arcada.display->println("Error");
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
    return;
  } else {
    sampling = true;
  }

  uint8_t pressed_buttons = arcada.readButtons();
  if (digitalRead( START_INPUT_PIN ) == LOW && sampling == false ) {
    tone(ARCADA_AUDIO_OUT, 4000, 100);
    arcada.display->println("Starting Capture");
    arcada.display->setTextColor(ARCADA_YELLOW);
    arcada.display->println("Sampling...");
    sampling = true;
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

    // Calculate date field
    uint32_t dateAndHour = (gps.getYear() * 8928) + ((gps.getMonth() - 1) * 744) + ((gps.getDay() - 1) * 24) + gps.getHour();
    if (gpsPreviousDateAndHour != dateAndHour) {
      gpsPreviousDateAndHour = dateAndHour;
      gpsSyncBits++;
    }

    // Calculate time field
    uint32_t timeSinceHourStart = (gps.getMinute() * 30000) + (gps.getSecond() * 500) + (gps.getMillisecond() / 2);

    // Calculate latitude and longitude
    uint32_t latitude = gps.getLatitude();
    uint32_t longitude = gps.getLongitude();

    // Calculate altitude, speed and bearing
    uint32_t altitude = gps.getAltitudeMSL() / 1000;
    uint32_t speed = gps.getGroundSpeed() / 1000;
    uint32_t bearing = gps.getHeading();

    // Create main data
    tempData[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
    tempData[1] = timeSinceHourStart >> 8;
    tempData[2] = timeSinceHourStart;
    tempData[3] = (((gps.getFixType() & 0x3) << 6) | ( gps.getSIV() & 0x3F ));
    tempData[4] = latitude >> 24;
    tempData[5] = latitude >> 16;
    tempData[6] = latitude >> 8;
    tempData[7] = latitude >> 0;
    tempData[8] = longitude >> 24;
    tempData[9] = longitude >> 16;
    tempData[10] = longitude >> 8;
    tempData[11] = longitude >> 0;
    tempData[12] = altitude >> 8;
    tempData[13] = altitude;
    tempData[14] = speed >> 8;
    tempData[15] = speed;
    tempData[16] = bearing >> 8;
    tempData[17] = bearing;
    tempData[18] = round(gps.getPDOP() * 10.f);
    tempData[19] = 0xFF; // Unimplemented

    // Notify main characteristics
    gpsMainCharacteristic.notify(tempData, 20);

    // Create time data
    tempData[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
    tempData[1] = dateAndHour >> 8;
    tempData[2] = dateAndHour;

    // Notify time characteristics
    gpsTimeCharacteristic.notify(tempData, 3);

  }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(mainService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  if ( debug ) {
    Serial.print("Connected to ");
    Serial.println(central_name);
  }

  arcada.display->setTextColor(ARCADA_BLUE);
  arcada.display->println("Connected");
  connected = true;
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  connected = false;
  arcada.display->setTextColor(ARCADA_RED);
  arcada.display->println("Disconnected");
  if ( debug ) {
    Serial.println();
    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  }
}
