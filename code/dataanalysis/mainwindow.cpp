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
#include <QWebEngineView>
#include <QtWebSockets/QWebSocketServer>
#include <QtWebSockets/QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>

using namespace QtCharts;


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    resize(1024, 768);

    int port = 4242;
    zerosampletime = 0.0f;
    QAction *action_Open = new QAction();
    action_Open->setText("&Open");
    connect(action_Open, SIGNAL(triggered()), this, SLOT(openFile()));

    webSocketServer = new QWebSocketServer(QStringLiteral("Chat Server"), QWebSocketServer::NonSecureMode, this);
    if (webSocketServer->listen(QHostAddress::LocalHost, port))
    {
        QTextStream(stdout) << "Chat Server listening on port " << port << '\n';
        connect(webSocketServer, &QWebSocketServer::newConnection, this, &MainWindow::onNewConnection);
    }


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

                QVBoxLayout * verticalChartLayout = new QVBoxLayout();
                chartHolder = new QTabWidget();

                timeLineSlider = new QSlider();
                {
                    timeLineSlider->setOrientation(Qt::Horizontal);
                    timeLineSlider->setTickInterval(5);
                    timeLineSlider->setSingleStep(1);
                    timeLineSlider->setPageStep(5);
                    timeLineSlider->setTickPosition(QSlider::TicksBothSides);
                    connect(timeLineSlider, SIGNAL(sliderMoved(int)), this, SLOT(sliderChanged(int)));

                }
                verticalChartLayout->addWidget( chartHolder );
                verticalChartLayout->addWidget(timeLineSlider);

                {
                    sizes.append(800);
                    QWidget * holderWidget = new QWidget();
                    holderWidget->setLayout( verticalChartLayout );
                    splitter->addWidget(holderWidget);
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

                    mapView = new QWebEngineView(this);
                    {
                        QWebEnginePage *page = mapView->page();
                        page->load(QUrl(QStringLiteral("file:///F:/Development/other/datalogger/code/dataanalysis/http/index.html")));

                        sideSizes.append(400);
                        sideSplitter->addWidget(mapView);
                    }

                    sideSplitter->setSizes(sideSizes);
                    splitter->addWidget(sideSplitter);
                }
                splitter->setSizes(sizes);
                verticalLayout->addWidget(splitter);
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

    float sampleTime = millis + zerosampletime;

    const int gpsIndex = FindGPSSampleIndex( sampleTime );

    if ( gpsIndex > -1 ) {

        QJsonObject coord;

        coord["lat"] = QString("%1").arg(gpsSamples[gpsIndex].lat);
        coord["lon"] = QString("%1").arg(gpsSamples[gpsIndex].lon);

        QJsonDocument saveDoc(coord);
        //test
        for (QWebSocket *pClient : qAsConst(clients)) {
            pClient->sendTextMessage(saveDoc.toJson());
        }
    }
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
                sample.lat = Sample_ReadFloat( data, idx );
                sample.lon = Sample_ReadFloat( data, idx );
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
    calculateTimeline();
}

void MainWindow::calculateTimeline() {
    const int accelCount = accelerometerSamples.length();
    const int magCount = magneticFieldSamples.length();
    const int gyroCount = gyroscopeSamples.length();
    const int atmostCount = atmosphericPressureSamples.length();
    const int humidCount = relativeHumiditySamples.length();
    const int tempCount = temperatureSamples.length();
    const int gpsCount = gpsSamples.length();
    const int buttonCount = buttonPressedSamples.length();

    zerosampletime = 999999999999.9f;
    if ( accelCount > 0 ) {
        zerosampletime = accelerometerSamples[0].timestamp < zerosampletime ? accelerometerSamples[0].timestamp : zerosampletime;
    }
    if ( magCount > 0 ) {
        zerosampletime = magneticFieldSamples[0].timestamp < zerosampletime ? magneticFieldSamples[0].timestamp : zerosampletime;
    }
    if ( gyroCount > 0 ) {
        zerosampletime = gyroscopeSamples[0].timestamp < zerosampletime ? gyroscopeSamples[0].timestamp : zerosampletime;
    }
    if ( atmostCount > 0 ) {
        zerosampletime = atmosphericPressureSamples[0].timestamp < zerosampletime ? atmosphericPressureSamples[0].timestamp : zerosampletime;
    }
    if ( humidCount > 0 ) {
        zerosampletime = relativeHumiditySamples[0].timestamp < zerosampletime ? relativeHumiditySamples[0].timestamp : zerosampletime;
    }
    if ( tempCount > 0 ) {
        zerosampletime = temperatureSamples[0].timestamp < zerosampletime ? temperatureSamples[0].timestamp : zerosampletime;
    }
    if ( gpsCount > 0 ) {
        zerosampletime = gpsSamples[0].timestamp < zerosampletime ? gpsSamples[0].timestamp : zerosampletime;
    }
    if ( buttonCount > 0 ) {
        zerosampletime = buttonPressedSamples[0].timestamp < zerosampletime ? buttonPressedSamples[0].timestamp : zerosampletime;
    }
    createCharts();
}

void MainWindow::createCharts() {
    marker = new QLineSeries();
    marker->append(0,0);

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
            series->append(temperatureSamples[idx].timestamp-zerosampletime,temperatureSamples[idx].temperature );
        }
        chart->addSeries(series);
        chart->addSeries( marker );
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
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
            series->append(accelerometerSamples[idx].timestamp-zerosampletime,accelerometerSamples[idx].acceleration[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = accelerometerSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(accelerometerSamples[idx].timestamp-zerosampletime,accelerometerSamples[idx].acceleration[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = accelerometerSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(accelerometerSamples[idx].timestamp-zerosampletime,accelerometerSamples[idx].acceleration[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
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
            series->append(gyroscopeSamples[idx].timestamp-zerosampletime,gyroscopeSamples[idx].gyro[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = gyroscopeSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(gyroscopeSamples[idx].timestamp-zerosampletime,gyroscopeSamples[idx].gyro[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = gyroscopeSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(gyroscopeSamples[idx].timestamp-zerosampletime,gyroscopeSamples[idx].gyro[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
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
            series->append(magneticFieldSamples[idx].timestamp-zerosampletime,magneticFieldSamples[idx].magnetic[0] );
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = magneticFieldSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(magneticFieldSamples[idx].timestamp-zerosampletime,magneticFieldSamples[idx].magnetic[1]);
        }
        chart->addSeries(series);

        series = new QLineSeries();
        numSamples = magneticFieldSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(magneticFieldSamples[idx].timestamp-zerosampletime,magneticFieldSamples[idx].magnetic[2]);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
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
            series->append(atmosphericPressureSamples[idx].timestamp-zerosampletime,atmosphericPressureSamples[idx].pressure);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
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
            series->append(relativeHumiditySamples[idx].timestamp-zerosampletime,relativeHumiditySamples[idx].humidity);
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
    }

    {
        QChartView * chartView = new QChartView();
        QChart * chart = new QChart();
        chart->legend()->hide();
        chart->setTitle("Lap Marker");
        chartView->setChart(chart);
        chartView->setRenderHint(QPainter::Antialiasing);
        chartHolder->addTab(chartView, "Lap Marker");

        QLineSeries * series = new QLineSeries();
        int numSamples = buttonPressedSamples.count();
        for ( int idx = 0; idx < numSamples; ++idx ){
            series->append(buttonPressedSamples[idx].timestamp-zerosampletime, (int)(buttonPressedSamples[idx].value));
        }
        chart->addSeries(series);
        chart->createDefaultAxes();
        QList<QAbstractAxis*> axis = chart->axes(Qt::Horizontal);
        axis[0]->setMin(0);
    }

    {
        if ( gpsSamples.length() > 0 ) {
            QJsonObject coord;
            coord["lat"] = QString("%1").arg(gpsSamples[0].lat);
            coord["lon"] = QString("%1").arg(gpsSamples[0].lon);
            QJsonDocument saveDoc(coord);
            for (QWebSocket *pClient : qAsConst(clients)) {
                pClient->sendTextMessage(saveDoc.toJson());
            }
        }
    }
    update();
}

static QString getIdentifier(QWebSocket *peer)
{
    return QStringLiteral("%1:%2").arg(peer->peerAddress().toString(), QString::number(peer->peerPort()));
}

void MainWindow::onNewConnection()
{
    auto pSocket = webSocketServer->nextPendingConnection();
    QTextStream(stdout) << getIdentifier(pSocket) << " connected!\n";
    pSocket->setParent(this);

    connect(pSocket, &QWebSocket::textMessageReceived, this, &MainWindow::processMessage);
    connect(pSocket, &QWebSocket::disconnected, this, &MainWindow::socketDisconnected);

    clients << pSocket;

    if ( gpsSamples.length() > 0 ) {

        QJsonObject coord;

        coord["lat"] = QString("%1").arg(gpsSamples[0].lat);
        coord["lon"] = QString("%1").arg(gpsSamples[0].lon);

        QJsonDocument saveDoc(coord);
        //test
        for (QWebSocket *pClient : qAsConst(clients)) {
            pClient->sendTextMessage(saveDoc.toJson());
        }
    }
}

void MainWindow::processMessage(const QString &message)
{
    //QWebSocket *pSender = qobject_cast<QWebSocket *>(sender());
}

void MainWindow::socketDisconnected()
{
    QWebSocket *pClient = qobject_cast<QWebSocket *>(sender());
    QTextStream(stdout) << getIdentifier(pClient) << " disconnected!\n";
    if (pClient)
    {
        clients.removeAll(pClient);
        pClient->deleteLater();
    }
}

int MainWindow::FindGPSSampleIndex( const float sampleTime ) {
    const int numGpsSample = gpsSamples.length();
    for ( int idx = 0; idx < numGpsSample; ++idx ) {
        if ( idx < numGpsSample-1 ) {
            if ( sampleTime >= gpsSamples[idx].timestamp && sampleTime < gpsSamples[idx+1].timestamp ) {
                return idx;
            }
        } else {
            return numGpsSample-1;
        }
    }
    return -1;
}
