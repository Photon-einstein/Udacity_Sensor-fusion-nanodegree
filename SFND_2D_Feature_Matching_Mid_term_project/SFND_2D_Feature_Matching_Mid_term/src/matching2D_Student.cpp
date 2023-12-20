#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    double t;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "MAT_BF matching with cross-check = "<<crossCheck<<std::endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        cout << "FLANN matching"<<std::endl;
    }
    
    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in descriptor
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout<<"(SEL_NN) with n = "<<matches.size()<<" matches in "<< 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // STUDENT TASK
        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8f;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        std::cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }
}


//// STUDENT ASSIGNMENT
//// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
//// -> BRIEF, ORB, FREAK, AKAZE, SIFT
// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (descriptorType.compare("BRIEF") == 0) {
        int bytes = 32; 
        bool useOrientation = false;
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        //extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes, useOrientation);
    } else if (descriptorType.compare("ORB") == 0) {
        int nFeatures = 500;
        float scaleFactor = 1.2f;
        int nLevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;

        extractor = cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    } else if (descriptorType.compare("FREAK") == 0) {
        bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4;
        const std::vector< int > selectedPairs;

        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs);
    } else if (descriptorType.compare("AKAZE") == 0) {
        cv::AKAZE::DescriptorType descriptorType = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptorSize = 0;
        int descriptorChannels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;

        extractor = cv::AKAZE::create(descriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);
    } else if (descriptorType.compare("SIFT") == 0) {
        int nfeatures = 0; 
        int nOctaveLayers = 3; 
        double contrastThreshold = 0.04;
        double edgeThreshold = 10; 
        double sigma = 1.6;

        extractor = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    } else {
        std::cerr<<"#### \""+descriptorType+"\" is an unknown detector type, please provide one of the following ones: "+
            "\"BRISK\", \"BRIEF\", \"ORB\", \"FREAK\", \"AKAZE\" or \"SIFT\"."<<std::endl;
        exit(1);
    } 


    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}


void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,  bool bVis, bool nonMaximumSupression)
{

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    double t = (double)cv::getTickCount();
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                // Remove multiple maximas in the same vicinity
                bool bOverlap = false;
                if(nonMaximumSupression == true) {
                    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                    {
                        double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                        if (kptOverlap > maxOverlap)
                        {
                            bOverlap = true;
                            // if overlap is >t AND response is higher for new kpt, replace old key point with new one
                            if (newKeyPoint.response > (*it).response)
                            {
                                *it = newKeyPoint;
                                break; // quit loop over keypoints
                            }
                        }
                    }
                }
                
                // only add new key point if no overlap has been found in previous NMS
                if (!bOverlap)
                {
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris Detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize keypoints
    if (bVis)
    {
        string windowName = "Harris Corner Detector Response Matrix";
        cv::namedWindow(windowName, 5);
        cv::imshow(windowName, dst_norm_scaled);
        cv::waitKey(0);

        windowName = "Harris Corner Detection Results - After Non-Maximum Supression";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis) {
    double t = (double)cv::getTickCount();
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType.compare("FAST") == 0) {
        // STUDENT CODE
        int threshold = 30;                                                              // difference between intensity of the central pixel and pixels of a circle around this pixel
        bool bNMS = true;                                                                // perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    } else if (detectorType.compare("BRISK") == 0) {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        detector = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (detectorType.compare("ORB") == 0) {
        int nFeatures = 500;
        float scaleFactor = 1.2;
        int nLevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;

        detector = cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    } else if (detectorType.compare("AKAZE") == 0) {
        cv::AKAZE::DescriptorType 	descriptorType = cv::AKAZE::DESCRIPTOR_MLDB;
        int 	descriptorSize = 0;
        int 	descriptorChannels = 3;
        float 	threshold = 0.001f;
        int 	nOctaves = 4;
        int 	nOctaveLayers = 4;
        cv::KAZE::DiffusivityType 	diffusivity = cv::KAZE::DIFF_PM_G2; 

        detector = cv::AKAZE::create(descriptorType, descriptorSize, descriptorChannels, threshold,nOctaves, nOctaveLayers, diffusivity);
    } else if (detectorType.compare("SIFT") == 0) {
        int nfeatures = 0; 
        int nOctaveLayers = 3; 
        double contrastThreshold = 0.04;
        double edgeThreshold = 10; 
        double sigma = 1.6;

        detector = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);      
    }

    t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType + " with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// auxiliary method
std::string convertStringToUpperCase(const char *p) {
    std::string s="";
    while(*p != '\0') {
        s+=toupper(*p);
        ++p;
    }
    return s;
}
