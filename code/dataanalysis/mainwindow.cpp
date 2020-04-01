#include "mainwindow.h"

#include <QApplication>
#include <QAction>
#include <QMenuBar>
#include <QMenu>
#include <QVBoxLayout>
#include <QSplitter>
#include <QChartView>
#include <QChart>
#include <QVideoWidget>
#include <QListView>
#include <QSlider>
#include <QStatusBar>
#include <QMediaPlayer>
#include <QFileInfo>
#include <QUrl>
#include <QFileDialog>
#include <QLineSeries>
#include <QScatterSeries>

using namespace QtCharts;

#include "SensorTypes.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    resize(1024, 768);

    QAction *action_Open = new QAction();
    action_Open->setText("&Open");
    connect(action_Open, SIGNAL(triggered()), this, SLOT(openFile()));

    QMenu *menu_File = menuBar()->addMenu("&File");
    {
        menu_File->addAction(action_Open);
    }

    statusBar();

    QWidget *centralwidget = new QWidget();
    {
        QVBoxLayout * verticalLayout = new QVBoxLayout();
        {
            QSplitter *splitter = new QSplitter();
            {
                QList<int> sizes;
                splitter->setOrientation(Qt::Horizontal);

                chartHolder = new QTabWidget();
                {
                    sizes.append(800);
                    splitter->addWidget(chartHolder);
                }

                QSplitter * sideSplitter = new QSplitter();
                {
                    sideSplitter->setOrientation(Qt::Vertical);
                    sizes.append(224);
                    QList<int> sideSizes;
                    QVideoWidget *videoStream = new QVideoWidget;
                    {
                        player = new QMediaPlayer;
                        player->setVideoOutput(videoStream);
                        connect(player, SIGNAL(durationChanged(qint64)), this, SLOT(videoDurationAdjusted(qint64)));

                        sideSizes.append(400);
                        sideSplitter->addWidget(videoStream);
                    }

                    QListView *dataPlotsList = new QListView();
                    {
                        sideSizes.append(400);
                        sideSplitter->addWidget(dataPlotsList);
                    }

                    sideSplitter->setSizes(sideSizes);
                    splitter->addWidget(sideSplitter);
                }
                splitter->setSizes(sizes);
                verticalLayout->addWidget(splitter);
            }

            timeLineSlider = new QSlider();
            {
                timeLineSlider->setOrientation(Qt::Horizontal);
                timeLineSlider->setTickInterval(5);
                timeLineSlider->setSingleStep(1);
                timeLineSlider->setPageStep(5);
                timeLineSlider->setTickPosition(QSlider::TicksBothSides);
                connect(timeLineSlider, SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));
                verticalLayout->addWidget(timeLineSlider);
            }

            centralwidget->setLayout(verticalLayout);
        }
        setCentralWidget(centralwidget);
    }

}


MainWindow::~MainWindow()
{
}

void MainWindow::openFile() {
    fileName = QFileDialog::getOpenFileName(this, tr("Open Telemetry"), "F:/datalog", tr("Datalog (*.json)"));
    QFileInfo info( fileName );

    QString name = info.completeBaseName().replace("_datalog","_video");
    QDir dir = info.dir();

    QString videoPath = dir.path() + "/videos/" + name + ".mp4";
    if ( QFileInfo( videoPath ).exists() ) {
        player->setMedia(QUrl::fromLocalFile(videoPath));
        player->play();
        player->pause();
        player->setPosition(0);
    }

    parseFile();
}

void MainWindow::sliderChanged(int value) {
    qint64 millis = value * 100; // convert value to millis

    qint64 numMinutes = millis / 1000 / 60;
    qint64 numSeconds = millis / 1000 % 60;
    qint64 numMillis = millis % 1000;

    player->setPosition( millis );

    QString timeValue = QString("Current Time: %1:%2.%3").arg(numMinutes,2,10,QChar('0')).arg(numSeconds,2,10,QChar('0')).arg(numMillis,3,10,QChar('0'));
    statusBar()->showMessage(timeValue);
}

void MainWindow::videoDurationAdjusted(qint64 duration) {
    timeLineSlider->setRange(0, duration / 100 );
    timeLineSlider->setValue(0);
}

void MainWindow::parseFile() {
    QFile dataFile(fileName);
    if ( dataFile.exists() == false ) {
        return;
    }

    if ( dataFile.open(QIODevice::ReadOnly) == false ) {
        return;
    }

    int fileLength = dataFile.size();
    char * data = new char[fileLength+1];
    memset(data, 0, fileLength+1);
    if ( dataFile.read(data,fileLength+1) != fileLength ) {
        return; // we didnt read all the data might have to chunk
    }

    QVector<AccelerometerSample> accelerometerSamples;
    QVector<MagneticFieldSample> magneticFieldSamples;
    QVector<GyroscopeSample> gyroscopeSamples;
    QVector<AtmosphericPressureSample> atmosphericPressureSamples;
    QVector<RelativeHumiditySample> relativeHumiditySamples;
    QVector<TemperatureSample> temperatureSamples;
    QVector<GPSSample> gpsSamples;
    QVector<ButtonPressedSample> buttonPressedSamples;

    for ( int idx = 0; idx <= fileLength; ++idx ) {
        if ( data[idx] != '@' ) {
            continue;
        }

        // sample start
        int type = Sample_ReadInt( data, idx );
        int timestamp = Sample_ReadInt( data, idx );

        switch (type) {
            case SENSOR_TYPE_ACCELEROMETER :
            {
                AccelerometerSample sample;
                sample.timestamp = timestamp;
                sample.acceleration[0] = Sample_ReadFloat( data, idx );
                sample.acceleration[1] = Sample_ReadFloat( data, idx );
                sample.acceleration[2] = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    accelerometerSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_MAGNETIC_FIELD :
            {
                MagneticFieldSample sample;
                sample.timestamp = timestamp;
                sample.magnetic[0] = Sample_ReadFloat( data, idx );
                sample.magnetic[1] = Sample_ReadFloat( data, idx );
                sample.magnetic[2] = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    magneticFieldSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_GYROSCOPE:
            {
                GyroscopeSample sample;
                sample.timestamp = timestamp;
                sample.gyro[0] = Sample_ReadFloat( data, idx );
                sample.gyro[1] = Sample_ReadFloat( data, idx );
                sample.gyro[2] = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    gyroscopeSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_PRESSURE :
            {
                AtmosphericPressureSample sample;
                sample.timestamp = timestamp;
                sample.pressure = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    atmosphericPressureSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_RELATIVE_HUMIDITY :
            {
                RelativeHumiditySample sample;
                sample.timestamp = timestamp;
                sample.humidity = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    relativeHumiditySamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_AMBIENT_TEMPERATURE :
            {
                TemperatureSample sample;
                sample.timestamp = timestamp;
                sample.temperature = Sample_ReadFloat( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    temperatureSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_GPS :
            {
                GPSSample sample;
                sample.timestamp = timestamp;
                sample.latitude = Sample_ReadFloat( data, idx );
                sample.longitude = Sample_ReadFloat( data, idx );
                sample.age = Sample_ReadInt( data, idx );
                sample.status = Sample_ReadByte( data, idx );

                if ( Sample_EndIsValid( data, idx ) ) {
                    gpsSamples.append(sample);
                }
                break;
            }
            case SENSOR_TYPE_BUTTON :
            {
                ButtonReleasedSample sample;
                sample.timestamp = timestamp;
                sample.value = Sample_ReadByte( data, idx );
                if ( Sample_EndIsValid( data, idx ) ) {
                    buttonPressedSamples.append(sample);
                }
                break;
            }
        }
    }


    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Temperature");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Temperature");

        QLineSeries * series = new QLineSeries();
        int numSamples = temperatureSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(temperatureSamples[idx].timestamp,temperatureSamples[idx].temperature );
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }

    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Accelerometer");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Accelerometer");

        QLineSeries * series = new QLineSeries();
        int numSamples = accelerometerSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(accelerometerSamples[idx].timestamp,accelerometerSamples[idx].acceleration[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = accelerometerSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(accelerometerSamples[idx].timestamp,accelerometerSamples[idx].acceleration[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = accelerometerSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(accelerometerSamples[idx].timestamp,accelerometerSamples[idx].acceleration[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }


    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Gyroscopes");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Gyroscopes");

        QLineSeries * series = new QLineSeries();
        int numSamples = gyroscopeSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(gyroscopeSamples[idx].timestamp,gyroscopeSamples[idx].gyro[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = gyroscopeSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(gyroscopeSamples[idx].timestamp,gyroscopeSamples[idx].gyro[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = gyroscopeSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(gyroscopeSamples[idx].timestamp,gyroscopeSamples[idx].gyro[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }


    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Magnetic Field");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Magnetic Field");

        QLineSeries * series = new QLineSeries();
        int numSamples = magneticFieldSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(magneticFieldSamples[idx].timestamp,magneticFieldSamples[idx].magnetic[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = magneticFieldSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(magneticFieldSamples[idx].timestamp,magneticFieldSamples[idx].magnetic[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = magneticFieldSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(magneticFieldSamples[idx].timestamp,magneticFieldSamples[idx].magnetic[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }

    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Atmospheric Presure");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Atmospheric Presure");

        QLineSeries * series = new QLineSeries();
        int numSamples = atmosphericPressureSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(atmosphericPressureSamples[idx].timestamp,atmosphericPressureSamples[idx].pressure);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }

    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Relative Humidity");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Relative Humidity");

        QLineSeries * series = new QLineSeries();
        int numSamples = relativeHumiditySamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(relativeHumiditySamples[idx].timestamp,relativeHumiditySamples[idx].humidity);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }

    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Lap Marker");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Lap Marker");

        QScatterSeries * series = new QScatterSeries();
        int numSamples = buttonPressedSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(buttonPressedSamples[idx].timestamp, (int)(buttonPressedSamples[idx].value));
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
    }
    update();
}


