#include "ytracker.h"

#include "opencvblobslib/library/blob.h"
#include "opencvblobslib/library/BlobResult.h"
#include <opencv2/opencv.hpp>

#include <QCoreApplication>


#include <QDebug>

// print the content of the mat
void debugMat(cv::Mat mat, int prec)
{
    for(int i=0; i<mat.rows; i++)
    {
        std::cout << "[";
        for(int j=0; j<mat.cols; j++)
        {
            std::cout << std::setprecision(prec) << mat.at<float>(i,j);
            if(j != mat.size().width-1)
                std::cout << ", ";
            else
                std::cout << "]" << std::endl;
        }
    }


}


YTracker::YTracker() :
    trk_id(1), streamer_ori("ori.jpg"), streamer_pro("pro.jpg")
{
    // load the config from xml file
    loadConfig();

    // init the background segmentation engine
    bgs = new PixelBasedAdaptiveSegmenter;

    tracker = new CTracker(0.2,0.5,60.0,30,30);



}

int YTracker::init()
{
    cap.open(0);
    if ( !cap.isOpened() )  // if not success, exit program
    {
        std::cout << "Cannot open the video file" << std::endl;
        return -1;
    }
    return 0;
}

YTracker::~YTracker()
{
    cap.release();
    delete bgs;
    delete tracker;
}

// this function is called from main/gui thread in a loop
// to process every frame

int YTracker::timerSlot()
{
    int key_code = process();
    if (key_code == -2 || key_code == 27)
    {
//        streamer_ori.thread->wait();
//        streamer_pro.thread->wait();
        QCoreApplication::instance()->quit();
    }
    return 0;
}

int YTracker::process()
{
    cv::Mat img_origin;
    bool bSuccess = cap.read(img_origin); // read a new frame from video
    if (!bSuccess)
        return -2;


    // crop the image and get only the roi
    cv::Mat img_input = img_origin(crop_rect).clone();
    //cv::Mat img_input = img_origin.clone();
    //cv::resize(img_input, img_input, cv::Size(160, 120));
    // do some linear adjustment to the image
    img_input.convertTo(img_input, -1, adj_alpha, adj_beta);
    // first find the foreground
    cv::Mat img_mask;
    bgs->process(img_input, img_mask);

    // perform morph operation
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17), cv::Point(8, 8));
    morphologyEx(img_mask, img_mask, cv::MORPH_CLOSE, kernel);
    // find the blobs in the foreground

    CBlobResult blobs(img_mask, cv::Mat(), num_cores);
    blobs.Filter(blobs, FilterAction::FLT_EXCLUDE, CBlobGetArea(), FilterCondition::FLT_LESS, blob_min_size);

    std::vector<cv::Point2d> centers;
    for (size_t step = 0; step != blobs.GetNumBlobs(); ++step)
    {
        cv::Point tmp_pt = blobs.GetBlob(step)->getCenter();

        centers.push_back(cv::Point2d(tmp_pt.x, tmp_pt.y));
    }


    for(int i=0; i<centers.size(); i++)
    {
        cv::circle(img_input,centers[i],3,Scalar(0,255,0),1,CV_AA);

    }

    cv::Scalar Colors[]={cv::Scalar(255,0,0),cv::Scalar(0,255,0),cv::Scalar(0,0,255),
                         cv::Scalar(255,255,0),cv::Scalar(0,255,255),cv::Scalar(255,0,255),
                         cv::Scalar(255,127,255),cv::Scalar(127,0,255),cv::Scalar(127,0,127)};

    //if(centers.size()>0)
    //{
    tracker->Update(centers);

    //cout << tracker->tracks.size()  << endl;

    for(int i=0;i<tracker->tracks.size();i++)
    {
        if(tracker->tracks[i]->trace.size()>1)
        {
            for(int j=0;j<tracker->tracks[i]->trace.size()-1;j++)
            {
                cv::line(img_input,tracker->tracks[i]->trace[j],tracker->tracks[i]->trace[j+1],Colors[tracker->tracks[i]->track_id%9],2,CV_AA);
            }
        }
        cv::putText(img_input, "C"+std::to_string(tracker->tracks[i]->track_id),
                    tracker->tracks[i]->prediction, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
    }
    //}

    imshow("origin", img_origin);
    imshow("after", img_input);
    // fill in a dummy count string here
    streamer_ori.send(img_origin, "null");
    // send the count of car with the processed image
    streamer_pro.send(img_input, std::to_string(CTrack::NextTrackID));

    return cv::waitKey(30);

    //showTracks(img_input);


}

// draw the bounding boxes on the input image and show it on the screen
void YTracker::showTracks(cv::Mat p_img)
{
//    cv::Mat img = p_img.clone();
//    for (std::vector<MotionTrack>::iterator iter = tracks.begin();
//         iter != tracks.end(); ++iter)
//    {
//        cv::rectangle(img, iter->bbox, cv::Scalar(255, 0, 0), 3);
//        cv::Point pt(iter->bbox.x + iter->bbox.width/2,
//                     iter->bbox.y + iter->bbox.height/2);
//        cv::putText(img, std::to_string(iter->id), pt, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));

//        cv::putText(img, "C"+std::to_string(iter->id), iter->pred_center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
//    }

//    imshow("show track", img);
}

// load the config file
void YTracker::loadConfig()
{
    cv::FileStorage fs = cv::FileStorage("config/config.xml", cv::FileStorage::READ);

    blob_min_size = fs["blob_min_size"];
    blob_max_size = fs["blob_max_size"];

    //debugTrack = cvReadIntByName(fs, 0, "debugTrack", false);
    //debugBlob = cvReadIntByName(fs, 0, "debugBlob", false);
    //showBlobMask = cvReadIntByName(fs, 0, "showBlobMask", false);
    //showOutput = cvReadIntByName(fs, 0, "showOutput", true);

    crop_rect.x = fs["crop_rect_x"];
    crop_rect.y = fs["crop_rect_y"];
    crop_rect.width = fs["crop_rect_w"];
    crop_rect.height = fs["crop_rect_h"];

    triger_line.x = fs["triger_line_x"];
    triger_line.y = fs["triger_line_y"];
    triger_line.width = fs["triger_line_w"];
    triger_line.height = fs["triger_line_h"];

    adj_alpha = fs["adjustment_alpha"];
    adj_beta = fs["adjustment_beta"];

    num_cores = fs["num_cores"];

    cost_no_blob = fs["cost_no_blob"];
    cost_no_track = fs["cost_no_track"];

    track_fadeout_time = fs["track_fadeout_time"];
    track_burnin_time = fs["track_burnin_time"];
    age_threshold = fs["age_threshold"];

    fs.release();
}



