#include <stdarg.h>
#include <Wire.h>
#include <Adafruit_Arcada.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h> //accel gyro temp
#include <Adafruit_LIS3MDL.h> // magnetos
#include <Adafruit_SHT31.h> // humidity
#include <Adafruit_BMP280.h> // temp baro
#include <SparkFun_Ublox_Arduino_Library.h> //gps
#include <bluefruit.h>
#include "PacketIdInfo.h"

//#define DEBUG 0

BLEService mainService = BLEService(0x00000001000000fd8933990d6f411ff8);

//SENSORS

volatile uint32_t pulses = 0;
uint32_t rpmPrevMillis = 0;
float rpmcalc = 0.0f;
const byte rpmpulsePin = 9;
const byte throttlePositionPin = 10;
const byte brakePositionPin = 4;

BLECharacteristic raceChronoSensorDataMainCharacteristic   = BLECharacteristic (0x01);
BLECharacteristic raceChronoSensorDataFilterCharacteristic = BLECharacteristic (0x02);

static const int CMD_DENY_ALL = 0;
static const int CMD_ALLOW_ALL = 1;
static const int CMD_ADD_PID = 2;
PacketIdInfo raceChronoPacketIdInfo;
bool raceChronoAllowUnknownPackets = false;


typedef struct
{
    uint32_t id;
    uint64_t data;
    uint64_t data2;
} diyRCMSG;

Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33; // accel gyro temp
Adafruit_LIS3MDL lis3mdl; // magneto
Adafruit_SHT31 sht30; // baro
Adafruit_BMP280 bmp280; // humidity

//GPS 

BLECharacteristic gpsMainCharacteristic = BLECharacteristic (0x03);
BLECharacteristic gpsTimeCharacteristic = BLECharacteristic (0x04);

SFE_UBLOX_GPS gps; //gps

const int GPS_SAMPLE_TIME = 50; //ms
//uint32_t prevGpsMillis = 0;
uint32_t gpsPreviousDateAndHour = 0;
uint8_t gpsSyncBits = 0;
uint8_t gpsData[20];

// SENSOR DATA FILTER
void raceChronoFilterWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 1) {
        return;
    }
    uint8_t command = data[0];
    switch (command) {
    case CMD_DENY_ALL:
        if (len == 1) {
            raceChronoPacketIdInfo.reset();
            raceChronoAllowUnknownPackets = false;
        }
        break;
    case CMD_ALLOW_ALL:
        if (len == 3) {
            raceChronoPacketIdInfo.reset();
            uint16_t notifyIntervalMs = data[1] << 8 | data[2];
            raceChronoPacketIdInfo.setDefaultNotifyInterval(notifyIntervalMs);
            raceChronoAllowUnknownPackets = true;
        }
        break;
    case CMD_ADD_PID:
        if (len == 7) {
            uint16_t notifyIntervalMs = data[1] << 8 | data[2];
            uint32_t pid = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];
            raceChronoPacketIdInfo.setNotifyInterval(pid, notifyIntervalMs);
        }
        break;
    default:
        break;
    }
}

//BLE setup
void bluetoothSetupMainService(void) {
    mainService.begin();
    raceChronoSensorDataMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    raceChronoSensorDataMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    raceChronoSensorDataMainCharacteristic.begin();
    raceChronoSensorDataFilterCharacteristic.setProperties(CHR_PROPS_WRITE);
    raceChronoSensorDataFilterCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    raceChronoSensorDataFilterCharacteristic.setWriteCallback(*raceChronoFilterWriteCallback);
    raceChronoSensorDataFilterCharacteristic.begin();
    gpsMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsMainCharacteristic.begin();
    gpsTimeCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsTimeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsTimeCharacteristic.begin();
}

void bluetoothStartAdvertising(void) {
    Bluefruit.setTxPower(+4);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(mainService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 160); // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

void connect_callback(uint16_t conn_handle) {
    arcada.display->setTextColor(ARCADA_BLUE);
    arcada.display->println("Connected");
    tone(ARCADA_AUDIO_OUT, 4000, 300);
    raceChronoPacketIdInfo.reset();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    raceChronoPacketIdInfo.reset();
    arcada.display->setTextColor(ARCADA_RED);
    arcada.display->println("Disconnected");
    tone(ARCADA_AUDIO_OUT, 3000, 300);
}

void bluetoothStart() {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setName("QuinlanSoftwareIMU");
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    bluetoothSetupMainService();
    bluetoothStartAdvertising(); 
}

// CLUE arcada setup
void arcadaSetup() {
    arcada.arcadaBegin();
    arcada.displayBegin();
    arcada.setBacklight(240);
    arcada.display->setCursor(0, 0);
    arcada.display->setTextWrap(true);
    arcada.display->setTextSize(2);
    arcada.filesysBegin();
}

// Sensor Setup
void rpm_interupt() {
    pulses++;
}

void sensorSetup() {
    pinMode(rpmpulsePin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rpmpulsePin), rpm_interupt, FALLING);
    pinMode(throttlePositionPin, INPUT);
    pinMode(brakePositionPin, INPUT);
    
    lsm6ds33.begin_I2C();       // accel gyro
    lis3mdl.begin_I2C();        // magneto
    sht30.begin(0x44);          // humidty temp
    bmp280.begin();             // presure 
}

// gps setup
void gpsSetup() {
    gps.begin(Wire, 0x42);
}

// sensor send to rachchrono time sliced
void sensorNotifyLatestPacket(const uint32_t id) {
    uint32_t ms = millis();
    unsigned long delta = ms - rpmPrevMillis;
    if (pulses > 1) {
        rpmPrevMillis = ms;
        // 0.5 is due to 4 stroke per rev with 2 ignitions per rev so 2 pulses per revolution so / 2 or * 0.5
        // 30000 = ( ( 1000ms / delta ) * pulses ) * 60 / 2 = ((1000 * 60 * 0.5) / delta) * pulses
        rpmcalc = ((30000.0f / (float)delta) * (float)pulses);
        pulses = 0;
    }


    diyRCMSG motoMessage;
    motoMessage.id = id;
    motoMessage.data = 0;
    motoMessage.data2 = 0;

    //humidity
    int humi = sht30.readHumidity();
    humi = map(humi, 0, 100, 0, 10); // map to calibration 
    humi = constrain(humi, 0, 10); // constrain to percentage range
    
    //temp
    int temp = bmp280.readTemperature();
   
    //brakepos
    int brakePos = analogRead(brakePositionPin);
    brakePos = map(brakePos, 100, 560, 0, 63); // map to calibration
    brakePos = constrain(brakePos, 0, 63); // constrain to percentage range
    
    //throttle
    int throttlePos = analogRead(throttlePositionPin);
    throttlePos = map(throttlePos, 70, 720, 0, 255); // map to calibration 
    throttlePos = constrain(throttlePos, 0, 255); // constrain to percentage range
    
    //rem
    int rpm = map(rpmcalc, 0, 16000, 0, 255);
    rpm = constrain(rpm, 0, 255); // constrain to percentage range

    //airpressure
    int pres = bmp280.readPressure() * 0.1f;
    
    sensors_event_t accel, gyro, mag;
    lsm6ds33.getEvent(&accel, &gyro, 0);
    lis3mdl.getEvent(&mag);

    int accelx = map(accel.acceleration.x, -16, +16, -127, +127);
    int accely = map(accel.acceleration.y, -16, +16, -127, +127);
    int accelz = map(accel.acceleration.z, -16, +16, -127, +127);
    accelx = constrain(accelx, -127, +127);
    accely = constrain(accely, -127, +127);
    accelz = constrain(accelz, -127, +127);

    int gyrox = map(gyro.gyro.x, -6, +6, -127, +127);
    int gyroy = map(gyro.gyro.y, -6, +6, -127, +127);
    int gyroz = map(gyro.gyro.z, -6, +6, -127, +127);
    gyrox = constrain(gyrox, -127, +127);
    gyroy = constrain(gyroy, -127, +127);
    gyroz = constrain(gyroz, -127, +127);

    int magx = map(mag.magnetic.x, -400, +400, -127, +127);
    int magy = map(mag.magnetic.y, -400, +400, -127, +127);
    int magz = map(mag.magnetic.z, -400, +400, -127, +127);
    magx = constrain(magx, -127, +127);
    magy = constrain(magy, -127, +127);
    magz = constrain(magz, -127, +127);

    motoMessage.data |= ((uint64_t)(humi        & 0x0F))        & 0x000000000000000F;
    motoMessage.data |= ((uint64_t)(temp        & 0x3F) << 4)   & 0x0000000000000FF0;
    motoMessage.data |= ((uint64_t)(brakePos    & 0x3F) << 10)  & 0x000000000000FF00;
    motoMessage.data |= ((uint64_t)(throttlePos & 0xFF) << 16)  & 0x0000000000FF0000;
    motoMessage.data |= ((uint64_t)(rpm         & 0xFF) << 24)  & 0x00000000FF000000;
    motoMessage.data |= ((uint64_t)(pres        & 0xFF) << 32)  & 0x000000FF00000000;
    motoMessage.data |= ((uint64_t)(accelx      & 0xFF) << 40)  & 0x0000FF0000000000;
    motoMessage.data |= ((uint64_t)(accely      & 0xFF) << 48)  & 0x00FF000000000000;
    motoMessage.data |= ((uint64_t)(accelz      & 0xFF) << 56)  & 0xFF00000000000000;
    motoMessage.data2 |= ((uint64_t)(gyrox      & 0xFF))        & 0x00000000000000FF;
    motoMessage.data2 |= ((uint64_t)(gyroy      & 0xFF) << 8)   & 0x000000000000FF00;
    motoMessage.data2 |= ((uint64_t)(gyroz      & 0xFF) << 16)  & 0x0000000000FF0000;
    motoMessage.data2 |= ((uint64_t)(magx       & 0xFF) << 24)  & 0x00000000FF000000;
    motoMessage.data2 |= ((uint64_t)(magy       & 0xFF) << 32)  & 0x000000FF00000000;
    motoMessage.data2 |= ((uint64_t)(magz       & 0xFF) << 40)  & 0x0000FF0000000000;
    

    // Notify
    raceChronoSensorDataMainCharacteristic.notify((uint8_t*)&motoMessage, sizeof(motoMessage));
}

void sensorLoop() {
    // received a packet
    PacketIdInfoItem* infoItem = raceChronoPacketIdInfo.findItem(0x42, raceChronoAllowUnknownPackets);
    if (infoItem && infoItem->shouldNotify()) {
        sensorNotifyLatestPacket(0x42);    
        infoItem->markNotified();
    }
}

void gpsLoop() {
    uint16_t year = gps.getYear();
    uint8_t month = gps.getMonth();
    uint8_t day = gps.getDay();
    uint8_t hour = gps.getHour();
    uint8_t minute = gps.getMinute();
    uint8_t second = gps.getSecond();
    uint16_t milliseconds = gps.getMillisecond();
    uint32_t latitude = gps.getLatitude();
    uint32_t longitude = gps.getLongitude();
    int32_t altitude = (uint32_t)(gps.getAltitudeMSL() * 0.001f);
    uint32_t speed = (uint32_t)(gps.getGroundSpeed() * 0.000001f);
    uint32_t bearing = (int32_t)(gps.getHeading() * 0.0000001f);
    uint8_t fix = gps.getFixType();
    uint8_t sats = gps.getSIV();

    // Calculate date field
    uint32_t dateAndHour = (year - 2000) * 8928 + (month - 1) * 744 + (day - 1) * 24 + hour;
    if (gpsPreviousDateAndHour != dateAndHour) {
        gpsPreviousDateAndHour = dateAndHour;
        gpsSyncBits++;
    }

    // Calculate time field
    uint32_t timeSinceHourStart = ((((minute * 60) + second) * 1000) + milliseconds) * 0.5f; // number of 2ms since hour start
    int16_t adjustedAltitude = ((altitude + 500) * 10) & 0x7FFF;

    // Create main data
    gpsData[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
    gpsData[1] = ((timeSinceHourStart >> 8) & 0xFF);
    gpsData[2] = (timeSinceHourStart & 0xFF);
    gpsData[3] = (((min(0x3, fix) & 0x3) << 6) | (min(0x3F, sats) & 0x3F));
    gpsData[4] = latitude >> 24;
    gpsData[5] = latitude >> 16;
    gpsData[6] = latitude >> 8;
    gpsData[7] = latitude >> 0;
    gpsData[8] = longitude >> 24;
    gpsData[9] = longitude >> 16;
    gpsData[10] = longitude >> 8;
    gpsData[11] = longitude >> 0;
    gpsData[12] = adjustedAltitude >> 8;
    gpsData[13] = adjustedAltitude;
    gpsData[14] = speed >> 8;
    gpsData[15] = speed;
    gpsData[16] = bearing >> 8;
    gpsData[17] = bearing;
    gpsData[18] = 0;
    gpsData[19] = 0xFF; // Unimplemented

    // Notify main characteristics
    gpsMainCharacteristic.notify(gpsData, 20);

    // Create time data
    gpsData[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
    gpsData[1] = dateAndHour >> 8;
    gpsData[2] = dateAndHour;

    // Notify time characteristics
    gpsTimeCharacteristic.notify(gpsData, 3);
}

void setup() {
#if DEBUG
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
#endif    

    analogWrite(LED_BUILTIN, 0);
    bluetoothStart();
    arcadaSetup();
    sensorSetup();
    gpsSetup();
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("Ready");
}

void loop() {
    sensorLoop();
    gpsLoop();
}
