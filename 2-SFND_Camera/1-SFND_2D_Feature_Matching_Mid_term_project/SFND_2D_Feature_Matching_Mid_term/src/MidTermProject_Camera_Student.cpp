/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <iterator>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

    /* INIT VARIABLES AND DATA STRUCTURES */

    string detectorType = "";
    string descriptorType = "";
    string matcherType = "";
    string descriptorCategory = "";
    string selectorType = "";
    
    set<string> detectorTypeSet({"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"});
    set<string> descriptorTypeSet({"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"});
    set<string> matcherTypeSet({"MAT_BF", "MAT_FLANN"});
    set<string> descriptorCategorySet({"DES_BINARY", "DES_HOG"});
    set<string> selectorTypeSet({"SEL_NN", "SEL_KNN"});
    
    bool bLogging = true;
    bool inputParameterError = false;
    bool bFocusOnVehicle = true; // only keep keypoints on the preceding vehicle

    // Input parameter testing
    if( argc != 6 || detectorTypeSet.count( convertStringToUpperCase(argv[1]) ) == 0 ||
        descriptorTypeSet.count( convertStringToUpperCase(argv[2]) ) == 0 ||
        matcherTypeSet.count( convertStringToUpperCase(argv[3]) ) == 0 ||
        descriptorCategorySet.count( convertStringToUpperCase(argv[4]) ) == 0 ||
        selectorTypeSet.count( convertStringToUpperCase(argv[5]) ) == 0) {
            inputParameterError = true;
    }


    if (argc != 6 || inputParameterError == true) {
        cerr << "\nError: Wrong input arguments received. Exiting the program...\n" << endl;
        cout << "Use: ./2D_feature_tracking [detectorType] [descriptorType] [matcherType] [descriptorCategory] [selectorType]" << endl;
        cout << "[detectorType]:\t\tSHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << endl;
        cout << "[descriptorType]:\tBRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << endl;
        cout << "[matcherType]:\t\tMAT_BF, MAT_FLANN" << endl;
        cout << "[descriptorCategory]:\tDES_BINARY, DES_HOG" << endl;
        cout << "[selectorType]:\t\tSEL_NN, SEL_KNN\n" << endl;
        exit(1);
    } else {
        detectorType = convertStringToUpperCase(argv[1]);
        descriptorType = convertStringToUpperCase(argv[2]);
        matcherType = convertStringToUpperCase(argv[3]);
        descriptorCategory = convertStringToUpperCase(argv[4]);
        selectorType = convertStringToUpperCase(argv[5]);
    }

    // create a folder to evaluation results
    if (bLogging) {
        system("clear");
        system("rm -rf ../results");
        system("mkdir -p ../results");
    }

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize matches between current and previous image
    bool bLimitKpts =false; // optional : limit number of keypoints (helpful for debugging and learning)
            
    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() >= dataBufferSize) {
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        cout << "\n#1 : LOAD IMAGE "<<imgIndex+1<<" INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        // TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        // -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0) {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        } else if (detectorType.compare("HARRIS") == 0) {
            detKeypointsHarris(keypoints, imgGray, false, true);
        } else if (   detectorType.compare("FAST") == 0 ||
                    detectorType.compare("BRISK") == 0 || 
                    detectorType.compare("ORB") == 0 ||
                    detectorType.compare("AKAZE") == 0 || 
                    detectorType.compare("SIFT") == 0) {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        } else {
            cerr << "#2 : DETECT KEYPOINTS failed. Wrong detectorType - " << detectorType << " received. Use one of the detectors: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << endl;
            exit(1);
        }

        std::fstream resultKeypoints;
        if (bLogging) {
            resultKeypoints.open("../results/" + detectorType + "-" + descriptorType + "-keypoints.txt", std::ios::app);
            resultKeypoints << "===>>>" << imgFullFilename << endl;
            resultKeypoints << "Detected " << keypoints.size() << " keypoints";
        }

        // TASK MP.3 -> only keep keypoints on the preceding vehicle
        cv::Rect vehicleRect(535, 180, 180, 150);
        vector<float> tempSize;
        vector<vector<float>> neighborhoodSize;
        if (bFocusOnVehicle) {
            vector<cv::KeyPoint> keypointsROI;
            for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
                if (vehicleRect.contains(it->pt)) {
                    keypointsROI.push_back(*it);
                } else {
                    // save neighborhood size
                    tempSize.push_back(it->size);
                }
            }
            keypoints = keypointsROI;
            cout << detectorType << " detector with n=" << keypoints.size() << " keypoints in the rectangle ROI" << endl;

            neighborhoodSize.push_back(tempSize);

            if (bLogging) {
                resultKeypoints << " and " << keypoints.size() << " of them are on the preciding vehicle" << endl;

                resultKeypoints << "Neighborbood Sizes:" << endl;
                std::ostream_iterator<int> outIterator(resultKeypoints, "\t");
                for (int i = 0; i < neighborhoodSize.size(); ++i) {
                    std::copy(neighborhoodSize.at(i).begin(), neighborhoodSize.at(i).end(), outIterator);
                    resultKeypoints << endl;
                }
                resultKeypoints.close();
            }
        }

        if (bLimitKpts) {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0) { 
                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        // TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        // -> BRIEF, ORB, FREAK, AKAZE, SIFT
        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        // wait until at least two images have been processed
        if (dataBuffer.size() > 1)  {

            /* MATCH KEYPOINT DESCRIPTORS */

            // TASK MP.5 -> add FLANN matching in file matching2D.cpp
            // TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            vector<cv::DMatch> matches;
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorCategory, matcherType, selectorType);

            std::fstream resultMatchedKeypoints;
            if (bLogging) {
                resultMatchedKeypoints.open("../results/" + detectorType + "-" + descriptorType + "-matchedkeypoints.txt", std::ios::app);
                resultMatchedKeypoints << "===>>>" << imgFullFilename << endl;
                resultMatchedKeypoints << "Extracted " << matches.size() << " matched keypoints" << endl;
                resultMatchedKeypoints.close();
            }

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            if (bVis) {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                if (bLogging) {
                    string resultImg = "../results/" + detectorType + "-" + descriptorType + "-img" + imgNumber.str() + imgFileType;
                    cv::imwrite(resultImg, matchImg);
                } else {
                    cout << "Press key to continue to next image" << endl;
                    cv::waitKey(0); // wait for key to be pressed
                }
            }
        }

    } // eof loop over all images

    return 0;
}