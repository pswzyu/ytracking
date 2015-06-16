//#include "mainwindow.h"
//#include <QApplication>


#include "tracker/ytracker.h"

#include "hungarian/src/munkres.h"


int main(int argc, char *argv[])
{
    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    //return a.exec();

//    cv::Mat_<int> a;
//    a <<

//    return 0;


    cv::VideoCapture cap("data/video.avi");

    if ( !cap.isOpened() )  // if not success, exit program
    {
        std::cout << "Cannot open the video file" << std::endl;
        return -1;
    }

    YTracker ytracker;

    int key = 0;
    while(key != 'q')
    {
        cv::Mat frame;
        bool bSuccess = cap.read(frame); // read a new frame from video

        imshow("Origin", frame);

        if (!bSuccess) //if not success, break loop
        {
            std::cout << "Cannot read the frame from video file" << std::endl;
            break;
        }

        if(cv::waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        {
            std::cout << "esc key is pressed by user" << std::endl;
            break;
        }

        ytracker.process(frame);

    }


    return 0;
}
