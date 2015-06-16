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
    int blob_min_size;
    int blob_max_size;
    double adj_alpha; // for the linear adjustment of the image
    double adj_beta;
    int num_cores;
    float cost_no_blob;
    float cost_no_track;


    YTracker();
    ~YTracker();

    void process(cv::Mat& img_origin);

private:
    void calculateAssignmentCosts(CBlobResult& blobs, std::vector<MotionTrack>& trks, cv::Mat& costs,
                                  float cost_no_blob, float cost_no_track);
    void predictNewLocationsOfTracks();
    void updateAssignedTracks(CBlobResult &blobs);
    void deleteLostTracks(CBlobResult &blobs);
    void createNewTracks(CBlobResult &blobs);
    void updateUnassignedTracks(CBlobResult &blobs);

    void loadConfig();


};

#endif // YTRACKER_H
