//#include "mainwindow.h"
#include <QCoreApplication>
#include <QTimer>


#include "tracker/ytracker.h"


int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    //return a.exec();

//    cv::Mat_<int> a;
//    a <<

//    return 0;


    //cv::VideoCapture cap("data/video.avi");

    QCoreApplication app(argc, argv);

    YTracker ytracker;
    if (ytracker.init() != 0)
    {
        return -1;
    }

    QTimer timer(&app);
    timer.setInterval(0);
    QObject::connect(&timer, SIGNAL(timeout()), &ytracker, SLOT(timerSlot()));
    timer.start();



    return app.exec();
    //return 0;
}
