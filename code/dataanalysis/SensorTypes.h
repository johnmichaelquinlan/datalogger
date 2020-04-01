#ifndef SENSORTYPES_H
#define SENSORTYPES_H

/** Sensor types */
typedef enum {
  SENSOR_TYPE_ACCELEROMETER = (1), /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD = (2),
  SENSOR_TYPE_ORIENTATION = (3),
  SENSOR_TYPE_GYROSCOPE = (4),
  SENSOR_TYPE_LIGHT = (5),
  SENSOR_TYPE_PRESSURE = (6),
  SENSOR_TYPE_PROXIMITY = (8),
  SENSOR_TYPE_GRAVITY = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION = (10), /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE = (13),
  SENSOR_TYPE_OBJECT_TEMPERATURE = (14),
  SENSOR_TYPE_VOLTAGE = (15),
  SENSOR_TYPE_CURRENT = (16),
  SENSOR_TYPE_COLOR = (17),
  SENSOR_TYPE_GPS = (19),
  SENSOR_TYPE_BUTTON = (20),
} sensors_type_t;

typedef struct ButtonPressedSample
{
    int timestamp;
    unsigned char value;
} ButtonReleasedSample;

typedef struct TemperatureSample
{
    int timestamp;
    float temperature;
} TemperatureSample;

typedef struct AccelerometerSample
{
    int timestamp;
    float acceleration[3];
} AccelerometerSample;

typedef struct GyroscopeSample
{
    int timestamp;
    float gyro[3];
} GyroscopeSample;

typedef struct MagneticFieldSample
{
    int timestamp;
    float magnetic[3];
} MagneticFieldSample;

typedef struct AtmosphericPressureSample
{
    int timestamp;
    float pressure;
} AtmosphericPressureSample;

typedef struct RelativeHumiditySample
{
    int timestamp;
    float humidity;
} RelativeHumiditySample;

typedef struct GPSSample
{
    int timestamp;
    float longitude;
    float latitude;
    unsigned int age;
    unsigned char status;
} GPSSample;


int Sample_ReadInt( char * buf, int & index ) {
    int temp = 0;
    temp |= 0x000000FF & (buf[++index]);
    temp |= 0x0000FF00 & (buf[++index] << 8);
    temp |= 0x00FF0000 & (buf[++index] << 16);
    temp |= 0xFF000000 & (buf[++index] << 24);
    return temp;
}

float Sample_ReadFloat( char * buf, int & index ) {
    int temp = Sample_ReadInt(buf,index);
    return *reinterpret_cast<float*>(&temp);
}

char Sample_ReadByte( char * buf, int & index ) {
    return buf[++index];
}

bool Sample_EndIsValid( char * buf, int & index ) {
    char sampleEnd = Sample_ReadByte( buf, index );
    return sampleEnd == '\n';
}

#endif // SENSORTYPES_H
