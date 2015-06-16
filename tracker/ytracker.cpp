#include "ytracker.h"

#include "opencvblobslib/library/blob.h"
#include "opencvblobslib/library/BlobResult.h"
#include <opencv2/opencv.hpp>

#include "hungarian/src/munkres.h"

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


YTracker::YTracker()
{
    // init the background segmentation engine
    bgs = new PixelBasedAdaptiveSegmenter;
    loadConfig();
}

YTracker::~YTracker()
{
    delete bgs;
}

// this function is called from main/gui thread in a loop
// to process every frame

void YTracker::process(cv::Mat &img_origin)
{
    // crop the image and get only the roi
    //cv::Mat img_input = img_origin(crop_rect).clone();
    cv::Mat img_input = img_origin.clone();
    // do some linear adjustment to the image
    img_input.convertTo(img_input, -1, adj_alpha, adj_beta);
    // first find the foreground
    cv::Mat img_mask;
    bgs->process(img_input, img_mask);
    // perform morph operation
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13), cv::Point(6, 6));
    morphologyEx(img_mask, img_mask, cv::MORPH_OPEN, kernel);
    // find the blobs in the foreground
    imshow("img_before_blob", img_mask);
    CBlobResult blobs(img_mask, cv::Mat(), num_cores);
    std::cout << "numb1:" << blobs.GetNumBlobs() << std::endl;
    blobs.Filter(blobs, FilterAction::FLT_EXCLUDE, CBlobGetArea(), FilterCondition::FLT_LESS, blob_min_size);

    predictNewLocationsOfTracks();

    std::cout << "numb2:" << blobs.GetNumBlobs() << std::endl;

    cv::Mat costs;
    calculateAssignmentCosts(blobs, tracks, costs, cost_no_blob, cost_no_track);

    //    % Step 2: Solve the assignment problem represented by the cost matrix using
    //    % the AssignmentProblemSolver function. The function takes the cost
    //    % matrix and the cost of not assigning any detections to a track.
    Munkres m;
    m.diag(false);
    cv::Mat_<int> int_costs;
    costs.convertTo(int_costs, CV_32S);
    m.solve(int_costs);

    // Display solved matrix.
    for ( int row = 0 ; row < int_costs.rows; row++ ) {
        for ( int col = 0 ; col < int_costs.cols ; col++ ) {
            std::cout.width(2);
            std::cout << int_costs(row,col) << ",";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;


    for ( int row = 0 ; row < int_costs.rows ; row++ ) {
        int rowcount = 0;
        for ( int col = 0 ; col < int_costs.cols ; col++  ) {
            if ( int_costs(row,col) == 0 )
                rowcount++;
        }
        if ( rowcount != 1 )
            std::cerr << "Row " << row << " has " << rowcount << " columns that have been matched." << std::endl;
    }

    for ( int col = 0 ; col < int_costs.cols ; col++ ) {
        int colcount = 0;
        for ( int row = 0 ; row < int_costs.rows ; row++ ) {
            if ( int_costs(row,col) == 0 )
                colcount++;
        }
        if ( colcount != 1 )
            std::cerr << "Column " << col << " has " << colcount << " rows that have been matched." << std::endl;
    }



    //updateAssignedTracks(blobs);
    //updateUnassignedTracks(blobs);
    //deleteLostTracks(blobs);
    //createNewTracks(blobs);

}

// use the kalman filter to predict the position of the object, stored in
// the bbox member of the MotionTrack

void YTracker::predictNewLocationsOfTracks()
{
    // treverse the vector of tracks, predict for each
    for (std::vector<MotionTrack>::iterator iter = tracks.begin();
         iter != tracks.end(); ++iter)
    {
        cv::Mat pred_center = iter->kf.predict();
        iter->bbox.x = pred_center.at<float>(0) - iter->bbox.width / 2;
        iter->bbox.y = pred_center.at<float>(0) - iter->bbox.height / 2;
    }
}

//%% Assign Detections to Tracks
//% Assigning object detections in the current frame to existing tracks is
//% done by minimizing cost. The cost is defined as the negative
//% log-likelihood of a detection corresponding to a track.
//%
//% The algorithm involves two steps:
//%
//% Step 1: Compute the cost of assigning every detection to each track using
//% the |distance| method of the |vision.KalmanFilter| System object(TM). The
//% cost takes into account the Euclidean distance between the predicted
//% centroid of the track and the centroid of the detection. It also includes
//% the confidence of the prediction, which is maintained by the Kalman
//% filter. The results are stored in an MxN matrix, where M is the number of
//% tracks, and N is the number of detections.

void YTracker::calculateAssignmentCosts(CBlobResult& blobs, std::vector<MotionTrack>& trks, cv::Mat& costs,
                                        float cost_no_blob, float cost_no_track)
{
    // get the size of the padded mat
    int size_padded = trks.size() + blobs.GetNumBlobs();
    // set the dim of the result mat
    costs.create(size_padded, size_padded, CV_32F);

    costs.setTo(99999); // for easy padding

    // calculate the cost of assignment
    for (size_t step1 = 0; step1 != trks.size(); ++ step1)
    {
        for (size_t step2 = 0; step2 != blobs.GetNumBlobs(); ++ step2)
        {
            int pred_x = trks[step1].bbox.x + trks[step1].bbox.width / 2;
            int pred_y = trks[step1].bbox.y + trks[step1].bbox.height / 2;
            CBlob* one_blob = blobs.GetBlob(step2);
            cv::Point det_center = one_blob->getCenter();
            costs.at<float>(step1, step2) = sqrtf( (pred_x - det_center.x)*(pred_x - det_center.x)
                                                   - (pred_y - det_center.y)*(pred_y - det_center.y) );
        }
    }
    // fill in dummy diag account for track that has no blob assigned to (top right corner of the padded matrix)
    int cur_row = 0;
    for (size_t step = blobs.GetNumBlobs(); step != size_padded; ++step)
    {
        costs.at<float>(cur_row, step) = cost_no_blob;
        ++cur_row;
    }
    // fill in dummy diag account for track that has no track assigned to (bottom left corner of the padded matrix)
    int cur_col = 0;
    for (size_t step = trks.size(); step != size_padded; ++step)
    {
        costs.at<float>(step, cur_col) = cost_no_track;
        ++cur_col;
    }
    // set bottom right corner to be all zero
    costs(cv::Rect(blobs.GetNumBlobs(), trks.size(), trks.size(), blobs.GetNumBlobs())).setTo(0);

    debugMat(costs, 3);
}

void YTracker::createNewTracks(CBlobResult& blobs)
{

}

void YTracker::deleteLostTracks(CBlobResult& blobs)
{

}

void YTracker::updateAssignedTracks(CBlobResult& blobs)
{

}

void YTracker::updateUnassignedTracks(CBlobResult& blobs)
{

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

    fs.release();
}



