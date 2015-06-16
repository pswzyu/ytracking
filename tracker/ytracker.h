#ifndef YTRACKER_H
#define YTRACKER_H

#include <vector>

#include "opencvblobslib/library/blob.h"
#include "opencvblobslib/library/BlobResult.h"

#include "opencv2/opencv.hpp"
#include "package_bgs/PBAS/PixelBasedAdaptiveSegmenter.h"

#include "tracker/motiontrack.h"

class YTracker
{
public:

    std::vector<MotionTrack> tracks;
    IBGS* bgs;

    cv::Rect crop_rect;
    cv::Rect triger_line;
    int trk_id;

    // config variables
    int blob_min_size;
    int blob_max_size;
    double adj_alpha; // for the linear adjustment of the image
    double adj_beta;
    int num_cores;
    float cost_no_blob;
    float cost_no_track;
    int track_fadeout_time;
    int track_burnin_time;
    int age_threshold;


    YTracker();
    ~YTracker();

    void process(cv::Mat& img_origin);

private:
    void calculateAssignmentCosts(CBlobResult& blobs, std::vector<MotionTrack>& trks, cv::Mat& costs,
                                  float cost_no_blob, float cost_no_track);
    void predictNewLocationsOfTracks();
    void updateAssignedTracks(CBlobResult &blobs, cv::Mat_<int> assignment);
    void deleteLostTracks(CBlobResult &blobs, cv::Mat_<int> assignment);
    void createNewTracks(CBlobResult &blobs, cv::Mat_<int> assignment);
    void updateUnassignedTracks(CBlobResult &blobs, cv::Mat_<int> assignment);

    void showTracks(cv::Mat img);

    void loadConfig();


};

#endif // YTRACKER_H
