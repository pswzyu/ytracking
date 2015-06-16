#include "motiontrack.h"

MotionTrack::MotionTrack(cv::Point init_pos) :
    age(1), total_visible_cnt(1), cons_inv_cnt(0)
{
    // init the kalman filter
    kf.init(4, 2, 0);
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    //cv::Mat_<float> measurement(2,1);
    //measurement.setTo(Scalar(0));

    // init...
    kf.statePre.at<float>(0) = init_pos.x;
    kf.statePre.at<float>(1) = init_pos.y;
    kf.statePre.at<float>(2) = 0;
    kf.statePre.at<float>(3) = 0;
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, cv::Scalar::all(.1));

//    kf.predict();
//    cv::Mat mm(2, 1, CV_32F);
//    mm.at<float>(0) = init_pos.x;
//    mm.at<float>(1) = init_pos.y;
//    kf.correct(mm);
}

MotionTrack::~MotionTrack()
{

}

