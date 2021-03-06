#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <uEye.h>

using namespace std;
using namespace cv;

// camera configuration parameters
const int img_width = 2560;
const int img_height = 1920;
int img_bpp = 24;
const HIDS hCamRight = 1;
const HIDS hCamLeft = 2;
HIDS hCam = -1;
int masterGain = 0;
int redGain = 5;
int greenGain = 1;
int blueGain = 15;
double exposureLeft = 98.0;
double exposureRight = 98.0;
void *pMemVoidRight = NULL;      // pointer to where the image is stored
void *pMemVoidLeft = NULL;       // pointer to where the image is stored
char *imgMemRight = NULL;
char *imgMemLeft = NULL;
int memIdRight;
int memIdLeft;
double disable = 0.0;

// openCV variables
const string &right_image_win = "Right Camera";
const string &left_image_win = "Left Camera";
const string &centroid_image_win = "Centroids";
Mat rightImageMat;      // the raw image data from the right camera
Mat rightGrayMat;       // grayscale version of the right camera image
Mat leftImageMat;       // the raw image data from the left camera
Mat leftGrayMat;        // grayscale version of the left camera image
//const int vertShift = 28;
const int vertShift = 0;
const int horzShift = 0;
const int vertLine = 550;
Mat rightImgWarpMat = (Mat_<double>(2,3) << 1, 0, horzShift, 0, 1, vertShift);
vector<Point> rightHarrisCornerPts;
vector<Point> leftHarrisCornerPts;
vector<Point> rightCornerPts;
vector<Point> leftCornerPts;
vector<vector<Point> > rightContours;
vector<vector<Point> > leftContours;
vector<Vec4i> rightHierarchy;
vector<Vec4i> leftHierarchy;
Mat dst_right, dst_right_norm, dst_right_norm_scaled;
Mat dst_left, dst_left_norm, dst_left_norm_scaled;
Size checkerSize(1, 1);

// Detector parameters
int blockSize = 8;
int apertureSize = 17;
double k = 0.04;

// other global variables
Point rightCenterPoint, rCenPtTop, rCenPtRight, rCenPtBottom, rCenPtLeft;
Point leftCenterPoint, lCenPtTop, lCenPtRight, lCenPtBottom, lCenPtLeft;
Point ptLCtr = Point(0, img_height/2);
Point ptLMinus = Point(0, vertLine);
Point ptRCtr = Point(img_width, img_height/2);
Point ptRMinus = Point(img_width, vertLine);
Point ptCTop = Point(img_width/2, 0);
Point ptCTopMinus = Point(img_width/2 - 100, 0);
Point ptCTopPlus = Point(img_width/2 + 100, 0);
Point ptCBot = Point(img_width/2, img_height);
Point ptCBotMinus = Point(img_width/2 - 100, img_height);
Point ptCBotPlus = Point(img_width/2 + 100, img_height);
double textFontSize = 2.2;
int lineWidth = 3;
double thresholdVal = 140.0;
int intThresholdVal = (int)thresholdVal;
bool isThreshold = false;
bool isCrosshairs = false;
bool isFindCorners = false;
bool isShowCorners = false;
bool isCornersFound = false;
bool isFindContours = false;
bool isShowContours = false;
bool isContoursFound = false;
bool isSaveLeft = false;
bool isSaveRight = false;
Scalar blue = Scalar(255, 0, 0);
Scalar green = Scalar(0, 255, 0);
Scalar red = Scalar(0, 0, 255);
Scalar yellow = Scalar(0, 255, 255);
Scalar teal = Scalar(255, 255, 0);
Scalar white = Scalar(255, 255, 255);
Scalar gray = Scalar(127, 127, 127);
double scaleFactor = 1.72;  // ratio of camera res to Lumus res

// other global constants
const double pi = 3.1415926;

void on_threshold_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isThreshold = true;
    }
    else if (buttonVal == 0) {
        isThreshold = false;
    }
}

void on_crosshairs_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isCrosshairs = true;
    }
    else if (buttonVal == 0) {
        isCrosshairs = false;
    }
}

void on_contour_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isFindContours = true;
    }
    else if (buttonVal == 0) {
        isFindContours = false;
        isContoursFound = false;
    }
}

void on_corners_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isFindCorners = true;
    }
    else if (buttonVal == 0) {
        isFindCorners = false;
        isCornersFound = false;
    }
}

void on_threshold_change(int threshVal, void *userVal)
{
    intThresholdVal = threshVal;
    thresholdVal = (double)intThresholdVal;
    stringstream threshSS;
    threshSS << intThresholdVal;
    string threshValStr;
    threshSS >> threshValStr;
    string threshStr = "new threshold value = ";
    threshStr.append(threshValStr);
    displayOverlay(right_image_win, threshStr, 1000);
}

void on_right_exposure_change(int exposureVal, void *userVal)
{
    exposureRight = (double)exposureVal;
    stringstream exposureRightSS;
    exposureRightSS << exposureRight;
    string exposureRightStr;
    exposureRightSS >> exposureRightStr;
    string exposureStr = "new right exposure value = ";
    exposureStr.append(exposureRightStr);
    displayOverlay(right_image_win, exposureStr, 1000);
    is_Exposure(hCamRight, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposureRight, sizeof(exposureRight));
}

void on_left_exposure_change(int exposureVal, void *userVal)
{
    exposureLeft = (double)exposureVal;
    stringstream exposureLeftSS;
    exposureLeftSS << exposureLeft;
    string exposureLeftStr;
    exposureLeftSS >> exposureLeftStr;
    string exposureStr = "new left exposure value = ";
    exposureStr.append(exposureLeftStr);
    displayOverlay(right_image_win, exposureStr, 1000);
    is_Exposure(hCamLeft, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposureLeft, sizeof(exposureLeft));
}

void on_save_left_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
    }
    else if (buttonVal == 0) {
        cout << "save left button clicked value = 0" << endl;
        isSaveLeft = true;
    }
}

void on_save_right_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
    }
    else if (buttonVal == 0) {
        cout << "save right button clicked value = 0" << endl;
        isSaveRight = true;
    }
}

void configureCamera(HIDS hCam, char *imgMem, int memId)
{
    if (is_InitCamera(&hCam, NULL) != IS_SUCCESS){
        cout << "could not get camera handle..." << endl;
    }

    // memory initialization
    is_AllocImageMem(hCam, img_width, img_height, img_bpp, &imgMem, &memId);

    // set memory active
    is_SetImageMem(hCam, imgMem, memId);

    // set display mode
    is_SetDisplayMode(hCam, IS_SET_DM_DIB);

    // disable auto parameters
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0);

    // set frame rate
    double newFPS = 0.0;
    double FPS = 10.0;
    is_SetFrameRate (hCam, FPS, &newFPS);
    cout << "frame rate set to " << newFPS << endl;

    // set the gain values for manual white balance
    is_SetHardwareGain(hCam, masterGain, redGain, greenGain, blueGain);

    // disable IR color correction
    double factors = 0.0;
    is_SetColorCorrection(hCam, IS_CCOR_DISABLE, &factors);

    // enable auto blacklevel
    uint8_t nMode = IS_AUTO_BLACKLEVEL_ON;
    is_Blacklevel(hCam, IS_BLACKLEVEL_CMD_SET_MODE, (void*)&nMode , sizeof(nMode ));

    // set auto black level offset
    uint8_t nOffset = 127;
    is_Blacklevel(hCam, IS_BLACKLEVEL_CMD_SET_OFFSET, (void*)&nOffset, sizeof(nOffset));
}

void findCentroid(vector<Point> corners, Point& center)
{
    int xMin = 10000;
    int xMax = -1;
    int yMin = 10000;
    int yMax = -1;
    vector<Point>::iterator pointIt;
    for (pointIt = corners.begin(); pointIt != corners.end(); ++pointIt) {
        Point tempPoint = *pointIt;
        if (xMin > tempPoint.x) {
            xMin = tempPoint.x;
        }
        if (xMax < tempPoint.x) {
            xMax = tempPoint.x;
        }
        if (yMin > tempPoint.y) {
            yMin = tempPoint.y;
        }
        if (yMax < tempPoint.y) {
            yMax = tempPoint.y;
        }
    }
    center.x = xMin + (xMax - xMin)/2;
    center.y = yMin + (yMax - yMin)/2;

}

double findAngle(Mat& imageMat, Point& center, int horizOffset, int& rY, int& lY)
{
    double base = (double)(horizOffset * 2);
    //
    // iterate up each side until white-black transition is found
    //
    // right...
    //
    rY = center.y;
    int rX = center.x + horizOffset;
    while ((uint)imageMat.at<uchar>(rY, rX) < 250) {
        --rY;
    }
    //
    // left...
    //
    lY = center.y;
    int lX = center.x - horizOffset;
    while ((uint)imageMat.at<uchar>(lY, lX) < 250) {
        --lY;
    }
    double height = (double)(rY - lY);
    cout << "base = " << base << ", height = " << height << endl;
    return (atan(height/base) * (180.0/pi));
}

int main(void)
{
    cout << "Hello OpenCV and IDS ueye on Linux!" << endl;

    //
    // configure cameras
    //
    configureCamera(hCamRight, imgMemRight, memIdRight);
    configureCamera(hCamLeft, imgMemLeft, memIdLeft);
    // adjust the gain values for to balance right and left cameras
    is_SetHardwareGain(hCamRight, masterGain + 1, redGain, greenGain, blueGain);
    is_SetHardwareGain(hCamLeft, masterGain, redGain, greenGain, blueGain);

    //
    // set the exposure time in msec
    //
    is_Exposure(hCamRight, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposureRight, sizeof(exposureRight));
    is_Exposure(hCamLeft, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposureLeft, sizeof(exposureLeft));

    //
    // create the image matrices and display windows
    //
    rightImageMat = Mat(Size(img_width, img_height), CV_8UC3);
    rightGrayMat = Mat(Size(img_width, img_height), CV_8U);
    leftImageMat = Mat(Size(img_width, img_height), CV_8UC3);
    leftGrayMat = Mat(Size(img_width, img_height), CV_8U);
    namedWindow(right_image_win, WINDOW_NORMAL | WINDOW_KEEPRATIO);
    namedWindow(left_image_win, WINDOW_NORMAL | WINDOW_KEEPRATIO);
    resizeWindow(right_image_win, 920, 690);
    resizeWindow(left_image_win, 920, 690);
    moveWindow(right_image_win, 985, 328);
    moveWindow(left_image_win, 0, 300);

    //
    // build up the utility control panel
    //
    createButton("Threshold", on_threshold_click, NULL, CV_CHECKBOX, 0);
    createButton("Crosshairs", on_crosshairs_click, NULL, CV_CHECKBOX, 0);
    createButton("Corners", on_corners_click, NULL, CV_CHECKBOX, 0);
    createButton("Contours", on_contour_click, NULL, CV_CHECKBOX, 0);
    createTrackbar("Threshold", "", &intThresholdVal, 255,
                   on_threshold_change, NULL);
    createTrackbar("Right Exposure", "", (int *)&exposureRight, 98,
                   on_right_exposure_change, NULL);
    createTrackbar("Left Exposure", "", (int *)&exposureLeft, 98,
                   on_left_exposure_change, NULL);
    createButton("Save Left Image", on_save_left_click, NULL, CV_PUSH_BUTTON, 0);
    createButton("Save Right Image", on_save_right_click, NULL, CV_PUSH_BUTTON, 0);

    //
    // link camera image buffers to OpenCV matrices
    //
    is_GetImageMem(hCamRight, &pMemVoidRight);    // get pointer to the image buffer
    is_GetImageMem(hCamLeft, &pMemVoidLeft);    // get pointer to the image buffer
    rightImageMat.data = (uchar *)pMemVoidRight;   // assign the pointer to Matrix
    leftImageMat.data = (uchar *)pMemVoidLeft;   // assign the pointer to Matrix

    //
    // start acquisition loop
    //
    int waitRet = -1;

    dst_right = Mat::zeros( rightGrayMat.size(), CV_32FC1 );

    while(1) {
        //
        // call is_FreezeVideo to grab a single frame from both cameras
        //
        if (is_FreezeVideo(hCamRight, IS_WAIT) == IS_SUCCESS &&
            is_FreezeVideo(hCamLeft, IS_WAIT) == IS_SUCCESS) {
            //
            // translate the right image vertically to align with the left image
            //
            warpAffine(rightImageMat, rightImageMat, rightImgWarpMat, rightImageMat.size());
            //
            // convert to binary threshold and find box corners...
            //
            if (isFindCorners) {
                cvtColor(rightImageMat, rightGrayMat, COLOR_BGR2GRAY);
                threshold(rightGrayMat, rightGrayMat, thresholdVal, 255, THRESH_BINARY);
                cvtColor(leftImageMat, leftGrayMat, COLOR_BGR2GRAY);
                threshold(leftGrayMat, leftGrayMat, thresholdVal, 255, THRESH_BINARY);

                //
                // detecting harris corners
                // finds corners from top to bottom of image
                //
                cout << "running cornerHarris..." << endl;
                cornerHarris( rightGrayMat, dst_right, blockSize,
                              apertureSize, k, BORDER_DEFAULT );
                cornerHarris( leftGrayMat, dst_left, blockSize,
                              apertureSize, k, BORDER_DEFAULT );

                //
                // Normalizing
                //
                normalize( dst_right, dst_right_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
                normalize( dst_left, dst_left_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
                convertScaleAbs( dst_right_norm, dst_right_norm_scaled );
                convertScaleAbs( dst_left_norm, dst_left_norm_scaled );

                //
                // add harris corners to vector...
                //
                cout << "finding right harris corners above threshold..." << endl;
                int rightHarrisCornerCount = 0;
                for( int j = 0; j < dst_right_norm.rows ; j++ ) {
                    for( int i = 0; i < dst_right_norm.cols; i++ ) {
                        if( (int)dst_right_norm.at<float>(j,i) > intThresholdVal ) {
                            ++rightHarrisCornerCount;
                            rightHarrisCornerPts.push_back(Point( i, j ));
                        }
                    }
                }
                //
                // add harris corners to vector...
                //
                cout << "finding left harris corners above threshold..." << endl;
                int leftHarrisCornerCount = 0;
                for( int j = 0; j < dst_left_norm.rows ; j++ ) {
                    for( int i = 0; i < dst_left_norm.cols; i++ ) {
                        if( (int)dst_left_norm.at<float>(j,i) > intThresholdVal ) {
                            ++leftHarrisCornerCount;
                            leftHarrisCornerPts.push_back(Point( i, j ));
                        }
                    }
                }

                cout << "found " << rightHarrisCornerCount << " right harris corners" << endl;
                cout << "found " << leftHarrisCornerCount << " left harris corners" << endl;

                //
                // find average right corners...
                //
                int cornerCount = 0;
                uint loopCount = 0;
                int repeatCount = 1;
                int xStart = 0;
                int yStart = 0;
                int xSum = 0;
                int ySum = 0;
                cout << "finding average of right harris corners..." << endl;
                while (loopCount < rightHarrisCornerPts.size()) {
                    //
                    // initialize starting values...
                    //
                    xStart = rightHarrisCornerPts[loopCount].x;
                    yStart = rightHarrisCornerPts[loopCount].y;
                    xSum += rightHarrisCornerPts[loopCount].x;
                    ySum += rightHarrisCornerPts[loopCount].y;
                    //
                    // iterate while x and y coordinates are close together
                    //
                    while ( ( abs(xStart - rightHarrisCornerPts[loopCount + 1].x) < 50)
                            &&
                            ( abs(yStart - rightHarrisCornerPts[loopCount + 1].y) < 50)
                            &&
                            loopCount < rightHarrisCornerPts.size()) {
                        //
                        // keep a running total...
                        //
                        ++loopCount;
                        ++repeatCount;
                        xSum += rightHarrisCornerPts[loopCount].x;
                        ySum += rightHarrisCornerPts[loopCount].y;
                    }
                    ++loopCount;
                    int x = xSum/repeatCount;
                    int y = ySum/repeatCount;
                    cout << "x = " << x << endl;
                    cout << "y = " << y << endl;
                    rightCornerPts.push_back(Point(x,y));
                    ++cornerCount;
                    cout << "corner count = " << cornerCount << endl;
                    xSum = 0;
                    ySum = 0;
                    repeatCount = 1;
                }

                cout << "final right corner count = " << cornerCount << endl;
                cout << "number of points in corner vector = " << rightCornerPts.size() << endl;

                //
                // find average left corners...
                //
                cornerCount = 0;
                loopCount = 0;
                repeatCount = 1;
                xStart = 0;
                yStart = 0;
                xSum = 0;
                ySum = 0;
                cout << "finding average of left harris corners..." << endl;
                while (loopCount < leftHarrisCornerPts.size()) {
                    //
                    // initialize starting values...
                    //
                    xStart = leftHarrisCornerPts[loopCount].x;
                    yStart = leftHarrisCornerPts[loopCount].y;
                    xSum += leftHarrisCornerPts[loopCount].x;
                    ySum += leftHarrisCornerPts[loopCount].y;
                    //
                    // iterate while x and y coordinates are close together
                    //
                    while ( ( abs(xStart - leftHarrisCornerPts[loopCount + 1].x) < 50)
                            &&
                            ( abs(yStart - leftHarrisCornerPts[loopCount + 1].y) < 50)
                            &&
                            loopCount < leftHarrisCornerPts.size()) {
                        //
                        // keep a running total...
                        //
                        ++loopCount;
                        ++repeatCount;
                        xSum += leftHarrisCornerPts[loopCount].x;
                        ySum += leftHarrisCornerPts[loopCount].y;
                    }
                    ++loopCount;
                    int x = xSum/repeatCount;
                    int y = ySum/repeatCount;
                    cout << "x = " << x << endl;
                    cout << "y = " << y << endl;
                    leftCornerPts.push_back(Point(x,y));
                    ++cornerCount;
                    cout << "corner count = " << cornerCount << endl;
                    xSum = 0;
                    ySum = 0;
                    repeatCount = 1;
                }
                isFindCorners = false;
                isShowCorners = true;
                isCornersFound = true;
            }
            //
            // apply threshold if selected...
            //
            else if (isThreshold) {
                cvtColor(rightImageMat, rightGrayMat, COLOR_BGR2GRAY);
                threshold(rightGrayMat, rightGrayMat, thresholdVal, 255, THRESH_BINARY);
                cvtColor(leftImageMat, leftGrayMat, COLOR_BGR2GRAY);
                threshold(leftGrayMat, leftGrayMat, thresholdVal, 255, THRESH_BINARY);
                //
                // draw cross hairs...
                //
                if (isCrosshairs) {
                    line(rightGrayMat, ptLCtr, ptRCtr, white, lineWidth);
                    line(rightGrayMat, ptCTop, ptCBot, white, lineWidth);
                    line(leftGrayMat, ptLCtr, ptRCtr, white, lineWidth);
                    line(leftGrayMat, ptCTop, ptCBot, white, lineWidth);
                }
                uchar pixelValue = rightGrayMat.at<uchar>(img_width/2, img_height/2);
                cout << "image pixel value at center = " << (uint)pixelValue << endl;
                pixelValue = rightGrayMat.at<uchar>(773, 1314);
                cout << "image pixel value at 773, 1314 = " << (uint)pixelValue << endl;

                imshow(right_image_win, rightGrayMat);
                imshow(left_image_win, leftGrayMat);
            }

            else if(isFindContours) {
                cvtColor(rightImageMat, rightGrayMat, COLOR_BGR2GRAY);
                cvtColor(leftImageMat, leftGrayMat, COLOR_BGR2GRAY);
                Canny(rightGrayMat, rightGrayMat, 100, intThresholdVal, 3);
                Canny(leftGrayMat, leftGrayMat, 100, intThresholdVal, 3);
                dilate(rightGrayMat, rightGrayMat, Mat(), Point(-1,-1));
                dilate(leftGrayMat, leftGrayMat, Mat(), Point(-1,-1));
                findContours( rightGrayMat, rightContours, rightHierarchy, CV_RETR_TREE,
                              CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
                findContours( leftGrayMat, leftContours, leftHierarchy, CV_RETR_TREE,
                              CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
                isFindContours = false;
                isShowContours = true;
                isContoursFound = true;
            }

            //
            // post processing and display stuff follows
            //
            else {
                //
                // Drawing a circle around corners and draw measurements
                //
                if (isShowCorners) {
                    int numCorners = rightCornerPts.size();
                    for (int i = 0; i < numCorners; i++) {
                        circle( rightImageMat, rightCornerPts[i], 20,  red, lineWidth, 8, 0 );
                        if (numCorners == 4) {
                            line(rightImageMat, rightCornerPts[0], rightCornerPts[1], red, lineWidth);
                            line(rightImageMat, rightCornerPts[0], rightCornerPts[2], red, lineWidth);
                            line(rightImageMat, rightCornerPts[2], rightCornerPts[3], red, lineWidth);
                            line(rightImageMat, rightCornerPts[3], rightCornerPts[1], red, lineWidth);
                        }
                    }
                    numCorners = leftCornerPts.size();
                    for (int i = 0; i < numCorners; i++) {
                        circle( leftImageMat, leftCornerPts[i], 20,  red, lineWidth, 8, 0 );
                        if (numCorners == 4) {
                            line(leftImageMat, leftCornerPts[0], leftCornerPts[1], red, lineWidth);
                            line(leftImageMat, leftCornerPts[0], leftCornerPts[2], red, lineWidth);
                            line(leftImageMat, leftCornerPts[2], leftCornerPts[3], red, lineWidth);
                            line(leftImageMat, leftCornerPts[3], leftCornerPts[1], red, lineWidth);
                        }
                    }
                    //
                    // calculate the tilt angles
                    //
                    double rightBase = (double)(rightCornerPts[0].x - rightCornerPts[1].x);
                    double rightHeight = (double)(rightCornerPts[0].y - rightCornerPts[1].y);
                    Point rightHoriz = Point(rightCornerPts[1].x, rightCornerPts[0].y);
                    line(rightImageMat, rightCornerPts[0], rightHoriz, green, lineWidth);
                    double rightAngle = atan(rightHeight/rightBase) * 180 / pi;

                    double leftBase = (double)(leftCornerPts[0].x - leftCornerPts[1].x);
                    double leftHeight = (double)(leftCornerPts[0].y - leftCornerPts[1].y);
                    Point leftHoriz = Point(leftCornerPts[1].x, leftCornerPts[0].y);
                    line(leftImageMat, leftCornerPts[0], leftHoriz, green, lineWidth);
                    double leftAngle = atan(leftHeight/leftBase) * 180 / pi;
                    //
                    // build up string to display angles
                    //
                    stringstream rightAngleSS;
                    rightAngleSS << rightAngle;
                    string rightAngleStr;
                    rightAngleSS >> rightAngleStr;
                    string rightAngleTextStr = "right angle = ";
                    rightAngleTextStr.append(rightAngleStr);

                    stringstream leftAngleSS;
                    leftAngleSS << leftAngle;
                    string leftAngleStr;
                    leftAngleSS >> leftAngleStr;
                    string leftAngleTextStr = "left angle = ";
                    leftAngleTextStr.append(leftAngleStr);
                    //
                    // display the angle strings
                    //
                    putText(rightImageMat, rightAngleTextStr,
                            Point(img_width/2 -325, img_height/2 - 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, blue, lineWidth);

                    putText(leftImageMat, leftAngleTextStr,
                            Point(img_width/2 -325, img_height/2 - 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, blue, lineWidth);
                    //
                    // find centroids
                    //
                    // left...
                    //
                    findCentroid(rightCornerPts, rightCenterPoint);
                    int rCrosshairWidth = abs(rightCornerPts[0].x - rightCornerPts[1].x);
                    int rCrosshairHeight = abs(rightCornerPts[0].y - rightCornerPts[3].y);
                    rCenPtTop.x = rightCenterPoint.x;
                    rCenPtTop.y = rightCenterPoint.y - rCrosshairHeight/2;
                    rCenPtRight.x = rightCenterPoint.x + rCrosshairWidth/2;
                    rCenPtRight.y = rightCenterPoint.y;
                    rCenPtBottom.x = rightCenterPoint.x;
                    rCenPtBottom.y = rightCenterPoint.y + rCrosshairHeight/2;
                    rCenPtLeft.x = rightCenterPoint.x - rCrosshairWidth/2;
                    rCenPtLeft.y = rightCenterPoint.y;
                    line(rightImageMat, rCenPtTop, rCenPtBottom, blue, lineWidth);
                    line(rightImageMat, rCenPtLeft, rCenPtRight, blue, lineWidth);
                    //
                    // right...
                    //
                    findCentroid(leftCornerPts, leftCenterPoint);
                    int lCrosshairWidth = abs(leftCornerPts[0].x - leftCornerPts[1].x);
                    int lCrosshairHeight = abs(leftCornerPts[0].y - leftCornerPts[3].y);
                    lCenPtTop.x = leftCenterPoint.x;
                    lCenPtTop.y = leftCenterPoint.y - lCrosshairHeight/2;
                    lCenPtRight.x = leftCenterPoint.x + lCrosshairWidth/2;
                    lCenPtRight.y = leftCenterPoint.y;
                    lCenPtBottom.x = leftCenterPoint.x;
                    lCenPtBottom.y = leftCenterPoint.y + rCrosshairHeight/2;
                    lCenPtLeft.x = leftCenterPoint.x - rCrosshairWidth/2;
                    lCenPtLeft.y = leftCenterPoint.y;
                    line(leftImageMat, lCenPtTop, lCenPtBottom, blue, lineWidth);
                    line(leftImageMat, lCenPtLeft, lCenPtRight, blue, lineWidth);
                    //
                    // build up strings to display
                    //
                    stringstream rCenterXSS;
                    rCenterXSS << rightCenterPoint.x;
                    string rCenterXStr;
                    rCenterXSS >> rCenterXStr;
                    stringstream rCenterYSS;
                    rCenterYSS << rightCenterPoint.y;
                    string rCenterYStr;
                    rCenterYSS >> rCenterYStr;
                    string rCenterTextStr = "center x = ";
                    rCenterTextStr.append(rCenterXStr);
                    rCenterTextStr.append(", center y = ");
                    rCenterTextStr.append(rCenterYStr);

                    stringstream lCenterXSS;
                    lCenterXSS << leftCenterPoint.x;
                    string lCenterXStr;
                    lCenterXSS >> lCenterXStr;
                    stringstream lCenterYSS;
                    lCenterYSS << leftCenterPoint.y;
                    string lCenterYStr;
                    lCenterYSS >> lCenterYStr;
                    string lCenterTextStr = "center x = ";
                    lCenterTextStr.append(lCenterXStr);
                    lCenterTextStr.append(", center y = ");
                    lCenterTextStr.append(lCenterYStr);
                    //
                    // display the strings
                    //
                    putText(rightImageMat, rCenterTextStr,
                            Point(img_width/2 - 550, img_height/2 + 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, blue, lineWidth);

                    putText(leftImageMat, lCenterTextStr,
                            Point(img_width/2 - 550, img_height/2 + 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, blue, lineWidth);
                    if (!isCornersFound) {
                        isShowCorners = false;
                    }
                }
                //
                // draw the contours on the images
                //
                else if(isShowContours) {
                    //
                    // right side...
                    //
                    double maxArea = -1.0;
                    int maxContourIndex = -1;
                    for( uint i = 0; i< rightContours.size(); i++ ) {
                        double temp = contourArea(rightContours.at(i));
                        if (maxArea < temp) {
                            maxArea = temp;
                            maxContourIndex = i;
                        }
                    }

                    //
                    // draw right crosshairs
                    //
                    Point rCenter, rPtCenterL, rPtCenterR, rPtCenterT, rPtCenterB;
                    findCentroid(rightContours.at(maxContourIndex), rCenter);

                    rightGrayMat = rightImageMat;
                    cvtColor(rightImageMat, rightGrayMat, COLOR_BGR2GRAY);
                    threshold(rightGrayMat, rightGrayMat, thresholdVal, 255, THRESH_BINARY);

                    uchar pixelValue = rightGrayMat.at<uchar>(img_width/2, img_height/2);
                    cout << "image pixel value at center = " << (uint)pixelValue << endl;
                    pixelValue = rightGrayMat.at<uchar>(773, 1314);
                    cout << "image pixel value at 773, 1314 = " << (uint)pixelValue << endl;
                    pixelValue = rightGrayMat.at<uchar>(rCenter.y, rCenter.x);
                    cout << "image pixel value at rCenter.y, rCenter.x = " << (uint)pixelValue << endl;

                    //
                    // calculate right tilt angle...
                    //

                    int horizOffset = 500;
                    int rrY;
                    int rlY;
                    double rightAngle = findAngle(rightGrayMat, rCenter, horizOffset, rrY, rlY);
                    cout << "right angle = " << rightAngle << endl;
                    Point rrAnglePt = Point(rCenter.x + horizOffset, rrY);
                    Point rlAnglePt = Point(rCenter.x - horizOffset, rlY);
                    line(rightImageMat, rlAnglePt, rrAnglePt, white, lineWidth);
                    Point rlAnglePtII = Point(rCenter.x - horizOffset, rrY);
                    line(rightImageMat, rlAnglePtII, rrAnglePt, red, lineWidth);

                    drawContours( rightImageMat, rightContours, maxContourIndex,
                                  blue, 2, 8, rightHierarchy, 0, Point() );

                    rPtCenterL = Point(0, rCenter.y);
                    rPtCenterR = Point(img_width, rCenter.y);
                    rPtCenterT = Point(rCenter.x, 0);
                    rPtCenterB = Point(rCenter.x, img_height);
                    line(rightImageMat, rPtCenterL, rPtCenterR, blue, lineWidth);
                    line(rightImageMat, rPtCenterT, rPtCenterB, blue, lineWidth);

                    //
                    // calculate the right horizontal vertical center distances
                    //
                    int rCenterDistX = rCenter.x - img_width/2;
                    int rCenterDistY = rCenter.y - img_height/2;


                    //
                    // left side...
                    //
                    maxArea = -1.0;
                    maxContourIndex = -1;
                    for( uint i = 0; i< leftContours.size(); i++ ) {
                        double temp = contourArea(leftContours.at(i));
                        if (maxArea < temp) {
                            maxArea = temp;
                            maxContourIndex = i;
                        }
                    }

                    //
                    // draw left crosshairs
                    //
                    Point lCenter, lPtCenterL, lPtCenterR, lPtCenterT, lPtCenterB;
                    findCentroid(leftContours.at(maxContourIndex), lCenter);

                    leftGrayMat = leftImageMat;
                    cvtColor(leftImageMat, leftGrayMat, COLOR_BGR2GRAY);
                    threshold(leftGrayMat, leftGrayMat, thresholdVal, 255, THRESH_BINARY);

                    uchar leftPixelValue = leftGrayMat.at<uchar>(img_width/2, img_height/2);
                    cout << "left image pixel value at center = " << (uint)leftPixelValue << endl;
                    leftPixelValue = leftGrayMat.at<uchar>(773, 1314);
                    cout << "left image pixel value at 773, 1314 = " << (uint)pixelValue << endl;
                    leftPixelValue = leftGrayMat.at<uchar>(lCenter.y, lCenter.x);
                    cout << "left image pixel value at lCenter.y, lCenter.x = " << (uint)leftPixelValue << endl;

                    //
                    // calculate left tilt angle...
                    //

                    int lrY;
                    int llY;
                    double leftAngle = findAngle(leftGrayMat, lCenter, horizOffset, lrY, llY);
                    cout << "left angle = " << leftAngle << endl;
                    Point lrAnglePt = Point(lCenter.x + horizOffset, lrY);
                    Point llAnglePt = Point(lCenter.x - horizOffset, llY);
                    line(leftImageMat, llAnglePt, lrAnglePt, white, lineWidth);
                    Point llAnglePtII = Point(lCenter.x - horizOffset, lrY);
                    line(leftImageMat, llAnglePtII, lrAnglePt, red, lineWidth);

                    drawContours( leftImageMat, leftContours, maxContourIndex,
                                  blue, 2, 8, leftHierarchy, 0, Point() );

                    lPtCenterL = Point(0, lCenter.y);
                    lPtCenterR = Point(img_width, lCenter.y);
                    lPtCenterT = Point(lCenter.x, 0);
                    lPtCenterB = Point(lCenter.x, img_height);
                    line(leftImageMat, lPtCenterL, lPtCenterR, blue, lineWidth);
                    line(leftImageMat, lPtCenterT, lPtCenterB, blue, lineWidth); 
                    //
                    // calculate the horizontal and vertical center distances
                    //
                    int lCenterDistX = lCenter.x - img_width/2;
                    int lCenterDistY = lCenter.y - img_height/2;


                    //
                    // build up strings to display
                    //
                    //
                    // calculations and display for the 4 arguments passed
                    // to the Python script
                    //
                    // arg1 == left shift vertical = .5 * (rCenter.y - lCenter.y)
                    // arg2 == left shift horizontal = rCenter.x - img_width/2
                    // arg3 == right shift vertical = .5 * (lCenter.y - rCenter.y)
                    // arg4 == right shift horizontal = lCenter.x - img_width/2
                    //
                    double rShiftVert = .5 * ((double)rCenter.y -
                                              (double)lCenter.y) / scaleFactor;
                    stringstream rShiftVertSS;
                    rShiftVertSS << (int)rShiftVert;
                    string rShiftVertStr;
                    rShiftVertSS >> rShiftVertStr;
                    string rVertStr = " arg1 (left vertical) = ";
                    rVertStr.append(rShiftVertStr);

                    double rShiftHoriz = (double)(rCenter.x - img_width/2)
                                            / scaleFactor;
                    stringstream rShiftHorizSS;
                    rShiftHorizSS << (int)rShiftHoriz;
                    string rShiftHorizStr;
                    rShiftHorizSS >> rShiftHorizStr;
                    string rHorizStr = " arg2 (left horizontal) = ";
                    rHorizStr.append(rShiftHorizStr);

                    double lShiftVert = .5 * ((double)lCenter.y -
                                              (double)rCenter.y) / scaleFactor;
                    stringstream lShiftVertSS;
                    lShiftVertSS << (int)lShiftVert;
                    string lShiftVertStr;
                    lShiftVertSS >> lShiftVertStr;
                    string lVertStr = " arg3 (right vertical) = ";
                    lVertStr.append(lShiftVertStr);

                    double lShiftHoriz = (double)(lCenter.x - img_width/2)
                                            / scaleFactor;
                    stringstream lShiftHorizSS;
                    lShiftHorizSS << (int)lShiftHoriz;
                    string lShiftHorizStr;
                    lShiftHorizSS >> lShiftHorizStr;
                    string lHorizStr = " arg4 (right horizontal) = ";
                    lHorizStr.append(lShiftHorizStr);

                    //
                    // display the strings near the crosshair on left side
                    //
                    if (rCenterDistX > 0) {
                        putText(leftImageMat, rVertStr,
                                Point(rCenter.x, rCenter.y -25), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    else {
                        putText(leftImageMat, rVertStr,
                                Point(rCenter.x, rCenter.y -25), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    if (rCenterDistY > 0) {
                        putText(leftImageMat, rHorizStr,
                                Point(rCenter.x, rCenter.y -60), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    else {
                        putText(leftImageMat, rHorizStr,
                                Point(rCenter.x, rCenter.y +45),
                                FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    }

                    //
                    // display the strings near the crosshair on right side
                    //
                    if (lCenterDistX > 0) {
                        putText(rightImageMat, lVertStr,
                                Point(lCenter.x, lCenter.y -25), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    else {
                        putText(rightImageMat, lVertStr,
                                Point(lCenter.x, lCenter.y -25), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    if (lCenterDistY > 0) {
                        putText(rightImageMat, lHorizStr,
                                Point(lCenter.x, lCenter.y -60), FONT_HERSHEY_SIMPLEX,
                                textFontSize, red, lineWidth);
                    }
                    else {
                        putText(rightImageMat, lHorizStr,
                                Point(lCenter.x, lCenter.y +45),
                                FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    }


                    //
                    // build up strings to display angles
                    //
                    stringstream rightAngleSS;
                    rightAngleSS << rightAngle;
                    string rightAngleStr;
                    rightAngleSS >> rightAngleStr;
                    string rightAngleTextStr = "right angle = ";
                    rightAngleTextStr.append(rightAngleStr);

                    stringstream leftAngleSS;
                    leftAngleSS << leftAngle;
                    string leftAngleStr;
                    leftAngleSS >> leftAngleStr;
                    string leftAngleTextStr = "left angle = ";
                    leftAngleTextStr.append(leftAngleStr);
                    //
                    // display the angle strings
                    //
                    putText(rightImageMat, rightAngleTextStr,
                            Point(img_width/2 -325, img_height/2 - 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, teal, lineWidth);

                    putText(leftImageMat, leftAngleTextStr,
                            Point(img_width/2 -325, img_height/2 - 500), FONT_HERSHEY_SIMPLEX,
                            textFontSize, yellow, lineWidth);


                    //
                    // remove overlays when done using them...
                    //
                    if (!isContoursFound) {
                        isShowContours = false;
                    }
                }
                //
                // draw cross hairs if desired...
                //
                if (isCrosshairs) {
                    line(rightImageMat, ptLCtr, ptRCtr, green, lineWidth);
                    line(rightImageMat, ptCTop, ptCBot, green, lineWidth);
                    line(leftImageMat, ptLCtr, ptRCtr, green, lineWidth);
                    line(leftImageMat, ptCTop, ptCBot, green, lineWidth);
                }
                //
                // save right image buffer
                //
                if (isSaveRight) {
                    //
                    // get date and time
                    //
                    time_t rawtime;
                    time (&rawtime);
                    string currentTime = ctime(&rawtime);
                    cout <<"The current local time is: " << currentTime << endl;
                    size_t found = currentTime.find(" ");
                    while (found != string::npos) {
                        currentTime.erase(found, 1);
                        found = currentTime.find(" ");
                    }
                    //
                    // remove last character
                    //
                    currentTime.erase(currentTime.length() - 1, 1);
                    cout <<"The current local time is: " << currentTime << endl;
                    //
                    // build file name
                    //
                    string fileName("rightCameraImage_");
                    fileName.append(currentTime);
                    fileName.append(".jpg");
                    cout << fileName << endl;
                    //
                    // build file path
                    //
                    string filePath = getenv("HOME");
                    filePath.append("/Pictures/");
                    filePath.append(fileName);
                    //
                    // buld up jpeg image data and write to file
                    //
                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(95);
                    try {
                        imwrite(filePath, rightImageMat, compression_params);
                        cout << "saving right image to file" << endl;
                    }
                    catch (Exception& ex) {
                        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
                        return 1;
                    }
                    isSaveRight = false;
                }
                //
                // save left image buffer
                //
                if (isSaveLeft) {
                    //
                    // get date and time
                    //
                    time_t rawtime;
                    time (&rawtime);
                    string currentTime = ctime(&rawtime);
                    cout <<"The current local time is: " << currentTime << endl;
                    size_t found = currentTime.find(" ");
                    while (found != string::npos) {
                        currentTime.erase(found, 1);
                        found = currentTime.find(" ");
                    }
                    //
                    // remove last character
                    //
                    currentTime.erase(currentTime.length() - 1, 1);
                    cout <<"The current local time is: " << currentTime << endl;
                    //
                    // build file name
                    //
                    string fileName("leftCameraImage_");
                    fileName.append(currentTime);
                    fileName.append(".jpg");
                    cout << fileName << endl;
                    //
                    // build file path
                    //
                    string filePath = getenv("HOME");
                    filePath.append("/Pictures/");
                    filePath.append(fileName);
                    //
                    // buld up jpeg image data and write to file
                    //
                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(95);
                    try {
                        imwrite(filePath, leftImageMat, compression_params);
                        cout << "saving left image to file" << endl;
                    }
                    catch (Exception& ex) {
                        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
                        return 1;
                    }
                    isSaveLeft = false;
                }
                //
                // all done processing
                // display images in windows
                //
                imshow(right_image_win, rightImageMat);
                imshow(left_image_win, leftImageMat);
            }
        }
        //
        // check if escape key pressed to terminate...
        //
        waitRet = waitKey(2);
        if (waitRet == 27) {
            break;
        }
    }
    destroyAllWindows();
    is_FreeImageMem(hCamRight, imgMemRight, memIdRight);
    is_ExitCamera(hCamRight);
    is_FreeImageMem(hCamLeft, imgMemLeft, memIdLeft);
    is_ExitCamera(hCamLeft);

    return 0;
}
