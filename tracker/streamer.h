#ifndef STREAMER_H
#define STREAMER_H

#include "opencv2/opencv.hpp"
#include <QMutex>
#include <string>

#include "streamthread.h"

class QNetworkAccessManager;

class Streamer : public QObject
{
    Q_OBJECT
public:
    bool is_working;
    QMutex lock;
    std::string filename;
    StreamThread* thread;
    cv::Mat img_to_send;
    std::string url;

    QNetworkAccessManager* mNetworkManager;


    Streamer(std::string p_fn);
    ~Streamer();

    void send(cv::Mat &img, std::string count);
    void uploadFile(std::string url, std::string fn);
    QByteArray buildUploadString(std::string fn, std::string count);

public slots:
    void printScriptReply(QNetworkReply* reply);
};

#endif // STREAMER_H
