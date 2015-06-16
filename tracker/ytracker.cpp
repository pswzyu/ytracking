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


YTracker::YTracker() :
    trk_id(1)
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
    CBlobResult blobs(img_mask, cv::Mat(), num_cores);
    blobs.Filter(blobs, FilterAction::FLT_EXCLUDE, CBlobGetArea(), FilterCondition::FLT_LESS, blob_min_size);

    predictNewLocationsOfTracks();

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

    // debug
    std::cout << "=========START=========" << std::endl;
    // show how many tracks and blobs are there
    std::cout << "Tracks: ";
    for (std::vector<MotionTrack>::iterator iter = tracks.begin();
         iter != tracks.end(); ++iter)
    {
        std::cout << iter->id << ", ";
    }
    std::cout << std::endl;
    // show how many blobs are there
    std::cout << "Blobs: " << blobs.GetNumBlobs() << std::endl;
    // show the cost matrix
    debugMat(costs, 3);

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



    updateAssignedTracks(blobs, int_costs);
    updateUnassignedTracks(blobs, int_costs);
    deleteLostTracks(blobs, int_costs);
    createNewTracks(blobs, int_costs);

    showTracks(img_input);

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
        iter->bbox.y = pred_center.at<float>(1) - iter->bbox.height / 2;

        iter->pred_center = cv::Point(pred_center.at<float>(0), pred_center.at<float>(1));
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
                                                   + (pred_y - det_center.y)*(pred_y - det_center.y) );
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
}

// this function finds the blob that is not assigned to any track, and add a track for it.
// means in work on the lower left corner of the solved matrix

void YTracker::createNewTracks(CBlobResult& blobs, cv::Mat_<int> assignment)
{
    // for each of the existing tracks, get the blob that is assigned to it
    // and use the info to update the info in the motion track obj
    int num_trk = tracks.size();
    for (size_t step1 = tracks.size(); step1 != num_trk+blobs.GetNumBlobs(); ++step1)
    {
        bool found = false; // used for debug
        for (size_t step2 = 0; step2 != blobs.GetNumBlobs(); ++step2)
        {
            if (assignment.at<int>(step1, step2) == 0)
            {
                found = true;
                // welcome the blob on index step2!
                // create a new track
                MotionTrack new_trk(blobs.GetBlob(step2)->getCenter());
                // set the bbox
                CvRect tmp_rect = blobs.GetBlob(step2)->GetBoundingBox();
                new_trk.bbox.x = tmp_rect.x;
                new_trk.bbox.y = tmp_rect.y;
                new_trk.bbox.width = tmp_rect.width;
                new_trk.bbox.height = tmp_rect.height;
                // set id
                new_trk.id = trk_id;
                // add the track to tracks
                tracks.push_back(new_trk);
                // update the global id counter
                ++trk_id;

            }
            // if not found, means no blob is assigned to this dummy track
            if (!found)
            {

            }
        }
    }
}

// go through all the tracks, find the dead tracks and delete them

void YTracker::deleteLostTracks(CBlobResult& blobs, cv::Mat_<int> assignment)
{
    for (std::vector<MotionTrack>::iterator iter = tracks.begin(); iter != tracks.end(); )
    {
        // age will not be 0
        if ( (iter->age < age_threshold && iter->total_visible_cnt / iter->age < 0.6) ||
             iter->cons_inv_cnt > track_fadeout_time )
        {
            // print some debug msg
            std::cout << "The track with id: " << iter->id << "is deleted!" << std::endl;

            iter = tracks.erase(iter);

        }else{
            ++iter;
        }
    }
}

// find the blobs that is assigned to a track, means do work on the top left
// corner of the solved matrix

void YTracker::updateAssignedTracks(CBlobResult& blobs, cv::Mat_<int> assignment)
{
    // for each of the existing tracks, get the blob that is assigned to it
    // and use the info to update the info in the motion track obj
    for (size_t step1 = 0; step1 != tracks.size(); ++step1)
    {
        bool found = false; // used for debug
        for (size_t step2 = 0; step2 != blobs.GetNumBlobs(); ++step2)
        {
            if (assignment.at<int>(step1, step2) == 0)
            {
                found = true;
                // the track's newest position is here!
                CvRect tmp_rect = blobs.GetBlob(step2)->GetBoundingBox();
                tracks[step1].bbox.x = tmp_rect.x;
                tracks[step1].bbox.y = tmp_rect.y;
                tracks[step1].bbox.width = tmp_rect.width;
                tracks[step1].bbox.height = tmp_rect.height;

                // get the center of the blob
                cv::Mat mm(2, 1, CV_32F);
                mm.at<float>(0) = blobs.GetBlob(step2)->getCenter().x;
                mm.at<float>(1) = blobs.GetBlob(step2)->getCenter().y;

                // update the kalman filter
                tracks[step1].kf.correct(mm);

                // update age of the track
                tracks[step1].age += 1;

                // update visibility related member
                tracks[step1].total_visible_cnt += 1;
                tracks[step1].cons_inv_cnt = 0;

            }
            // if not found, say something
            if (!found)
            {
                std::cout << "no real blob for track " << tracks[step1].id << std::endl;
            }
        }
    }
}

// this function find the tracks that has no blob assigned to, mark it
// as invisible and increase the age by 1, mean do work on the top right
// corner of the solved matrix

void YTracker::updateUnassignedTracks(CBlobResult& blobs, cv::Mat_<int> assignment)
{
    for (size_t step1 = 0; step1 != tracks.size(); ++step1)
    {
        bool found = false; // used for debug
        for (size_t step2 = blobs.GetNumBlobs(); step2 != blobs.GetNumBlobs()+tracks.size(); ++step2)
        {
            if (assignment.at<int>(step1, step2) == 0)
            {
                found = true;
                // update age of the track
                tracks[step1].age += 1;

                // update visibility related member
                tracks[step1].cons_inv_cnt += 1;

            }
            // if not found, means this track has no dummy blob assigned to it
            if (!found)
            {
                std::cout << "no dummy blob for track " << tracks[step1].id << std::endl;
            }
        }
    }
}

// draw the bounding boxes on the input image and show it on the screen
void YTracker::showTracks(cv::Mat p_img)
{
    cv::Mat img = p_img.clone();
    for (std::vector<MotionTrack>::iterator iter = tracks.begin();
         iter != tracks.end(); ++iter)
    {
        cv::rectangle(img, iter->bbox, cv::Scalar(255, 0, 0), 3);
        cv::Point pt(iter->bbox.x + iter->bbox.width/2,
                     iter->bbox.y + iter->bbox.height/2);
        cv::putText(img, std::to_string(iter->id), pt, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));

        cv::putText(img, "C"+std::to_string(iter->id), iter->pred_center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
    }

    imshow("show track", img);
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



