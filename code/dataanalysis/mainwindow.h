#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class QMediaPlayer;
class QMediaPlaylist;
class QSlider;
class QTabWidget;

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

private:
    void parseFile();

private:
    QString fileName;
    QMediaPlayer * player;
    QSlider * timeLineSlider;
    QTabWidget * chartHolder;
};
#endif // MAINWINDOW_H
