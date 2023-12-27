# SFND 2D Feature Tracking - Mid term report

## **MP.1 Data Buffer Optimization**
    This topic was address by means of testing if the dataBuffer vector already has 
    at least the same or more than dataBufferSize elements, if yes then the oldest 
    DataFrame is deleted and the new one is inserted, ensuring by this mean that we 
    do not have more than dataBufferSize elements at any time in dataBuffer vector.

## **MP.2 Keypoint Detection**
    All the requested detectors were implemented at the "matching2D_Student.cpp"

## **MP.3 Keypoint Removal** 
    All the keypoints were tested against a predefined rectangle, if the coordinates 
    of the keypoints are inside the rectangle then the keypoint is pushed into a vector 
    and processed further, if not then it will not be considered for the next processing.

## **MP.4 Keypoint Descriptors**
    The required descriptors were added in the method "descKeypoints", in the file 
    "matching2D_Student.cpp".

## **MP.5 Descriptor Matching**
    FLANN matching and K-nearest neighbor were implemented in the method "matchDescriptors", 
    in the file "matching2D_Student.cpp".

## **MP.6 Descriptor Distance Ratio** 
    The descriptor distance ration test was implemented at the file "matching2D_Student.cpp".

## **MP.7 Performance Evaluation 1**
    The data registered is available at the spreadsheet.

## **MP.8 Performance Evaluation 2**
    The data registered is available at the spreadsheet.

## **MP.9 Performance Evaluation 3**
    The data registered is available at the spreadsheet.

## **Top 3 recomendations:**

### **1: SHITOMASI detector with BRIEF descriptor**
    a) Most of the keypoints lied inside the vehicle. Highest Matched / detected ratio.  
    b) Also with BRIEF descriptor it is very fast, so suitable to our needs.

### **2: FAST detector with BRIEF descriptor**
    a) High number of keypoints detected. Second highest Matched / detected ratio.  
    b) Also with BRIEF descriptor it is very fast, so suitable to our needs.

### **3: BRISK detector with BRIEF descriptor**
    a) Highest number of keypoints detected. Third highest Matched / detected ratio.  
    b) Also with BRIEF descriptor it is very fast, so suitable to our needs.
