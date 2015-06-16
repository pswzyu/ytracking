#ifndef MOTIONTRACK_H
#define MOTIONTRACK_H

#include <opencv2/opencv.hpp>

class MotionTrack
{
public:
    int id;
    cv::Rect bbox;
    cv::KalmanFilter kf;
    int age;
    int total_visible_cnt;
    int cons_inv_cnt;

    cv::Point pred_center;



    MotionTrack(cv::Point init_pos);
    ~MotionTrack();
};

#endif // MOTIONTRACK_H
