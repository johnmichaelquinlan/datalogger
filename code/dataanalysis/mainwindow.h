#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineSeries>

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
    float lat;
    float lon;
    int age;
    int status;
} GPSSample;


class QMediaPlayer;
class QMediaPlaylist;
class QSlider;
class QTabWidget;
class QWebEngineView;

class QWebSocketServer;
class QWebSocket;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public Q_SLOTS:
    void openFile();
    void sliderChanged(int value);
    void videoDurationAdjusted(qint64 duration);
    void onNewConnection();
    void processMessage(const QString &message);
    void socketDisconnected();

private:
    void parseFile();
    void calculateTimeline();
    void createCharts();

    int Sample_ReadInt( char * buf, int & index ) {
        int temp = 0;
        temp |= 0x000000FF & (buf[++index]);
        temp |= 0x0000FF00 & (buf[++index] << 8);
        temp |= 0x00FF0000 & (buf[++index] << 16);
        temp |= 0xFF000000 & (buf[++index] << 24);
        return temp;
    }

    char * Sample_ReadString( char * buf, int & index, int length ) {
        char * buffer = new char[length+1];
        for ( int idx = 0; idx < length; ++idx ) {
            buffer[idx] = buf[++index];
        }
        buffer[length] = '\0';
        return buffer;
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

    int FindGPSSampleIndex( const float sampleTime );

private:
    QString fileName;
    QMediaPlayer * player;
    QSlider * timeLineSlider;
    QTabWidget * chartHolder;
    QWebEngineView * mapView;
    QtCharts::QLineSeries * marker;
    QWebSocketServer * webSocketServer;
    QList<QWebSocket *> clients;
    QVector<AccelerometerSample> accelerometerSamples;
    QVector<MagneticFieldSample> magneticFieldSamples;
    QVector<GyroscopeSample> gyroscopeSamples;
    QVector<AtmosphericPressureSample> atmosphericPressureSamples;
    QVector<RelativeHumiditySample> relativeHumiditySamples;
    QVector<TemperatureSample> temperatureSamples;
    QVector<GPSSample> gpsSamples;
    QVector<ButtonPressedSample> buttonPressedSamples;
    float zerosampletime;
};
#endif // MAINWINDOW_H
