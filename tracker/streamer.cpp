#include "streamer.h"

#include "streamthread.h"

#include <QUrl>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QFile>
#include <QByteArray>

Streamer::Streamer(std::string p_fn) :
    is_working(false), filename(p_fn), thread(NULL)
{
    mNetworkManager = new QNetworkAccessManager(this);
    connect(mNetworkManager, SIGNAL(finished(QNetworkReply*)), this, SLOT(printScriptReply(QNetworkReply*))); //This slot is used to debug the output of the server script

    url = "http://glhf.cnzy.me/epic_car/upload.php";
}

Streamer::~Streamer()
{
//    if (thread)
//    {
//        thread->wait();
//        thread->deleteLater();
//    }
}

void Streamer::send(cv::Mat& img, std::string count)
{
    lock.lock();

    if (is_working){
        lock.unlock();
        return;
    }

    is_working = true;

    img_to_send = img.clone();
    cv::imwrite(filename, img_to_send);

    lock.unlock();

//    if (thread)
//    {
//        thread->wait();
//        thread->deleteLater();
//    }

//    thread = new StreamThread(this, img, filename);
//    thread->start();

    //st.start();

    QByteArray postData;
    //Look below for buildUploadString() function
    postData = buildUploadString(filename, count);

    QUrl mResultsURL = QUrl(QString(url.c_str()));


    QString bound="yuzhanghaha"; //name of the boundary

    QNetworkRequest request(mResultsURL); //our server with php-script
    request.setRawHeader(QString("Content-Type").toLatin1(),QString("multipart/form-data; boundary=" + bound).toLatin1());
    request.setRawHeader(QString("Content-Length").toLatin1(), QString::number(postData.length()).toLatin1());


    mNetworkManager->post(request,postData);

}




QByteArray Streamer::buildUploadString(std::string fn, std::string count)
{
    QString path(fn.c_str());
    //path.append("\\\\");
    //path.append(getReportFileName());


    QString bound="yuzhanghaha";
    QByteArray data(QString("--" + bound + "\r\n").toLatin1());
    data.append("Content-Disposition: form-data; name=\"action\"\r\n\r\n");
    data.append("upload.php\r\n");
    data.append(QString("--" + bound + "\r\n").toLatin1());
    data.append("Content-Disposition: form-data; name=\"count\"\r\n\r\n");
    data.append(count.c_str());
    data.append("\r\n");
    data.append(QString("--" + bound + "\r\n").toLatin1());
    data.append("Content-Disposition: form-data; name=\"uploaded\"; filename=\"");
    data.append(fn.c_str());
    data.append("\"\r\n");
    data.append("Content-Type: image/jpg\r\n\r\n"); //data type

    QFile file(path);
        if (!file.open(QIODevice::ReadOnly)){
            //qDebug() << "QFile Error: File not found!";
            return data;
        } else {
            //qDebug() << "File found, proceed as planned";
        }

    data.append(file.readAll());
    data.append("\r\n");
    data.append("--" + bound + "--\r\n");  //closing boundary according to rfc 1867


    file.close();

    return data;
}







void Streamer::printScriptReply(QNetworkReply* reply)
{
//    qDebug() << "HERE";
//    QByteArray bytes = reply->readAll();
//    QString str = QString::fromUtf8(bytes.data(), bytes.size());
//    int statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();

//    qDebug() << QVariant(statusCode).toString();
//    qDebug() << str;
    reply->deleteLater();

    lock.lock();
    is_working = false;
    lock.unlock();
}
