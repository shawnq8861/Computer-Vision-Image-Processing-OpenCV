#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
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
double exposure = 98.0;
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
const int vertShift = 27;
//const int vertShift = 0;
const int horzShift = 0;
const int vertLine = 550;
Mat M = (Mat_<double>(2,3) << 1, 0, horzShift, 0, 1, vertShift);
vector<Point> rightHarrisCornerPts;
vector<Point> leftHarrisCornerPts;
vector<Point> rightCornerPts;
vector<Point> leftCornerPts;
Mat dst_right, dst_right_norm, dst_right_norm_scaled;
Mat dst_left, dst_left_norm, dst_left_norm_scaled;
Size checkerSize(1, 1);
bool cornersFound = false;

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
double thresholdVal = 125.0;
int intThresholdVal = (int)thresholdVal;
bool isThreshold = false;
bool isCrosshairs = false;
bool isCornersFound = false;
Scalar blue = Scalar(255, 0, 0);
Scalar green = Scalar(0, 255, 0);
Scalar red = Scalar(0, 0, 255);
Scalar white = Scalar(255, 255, 255);
Scalar gray = Scalar(127, 127, 127);

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

    // set the exposer time in msec
    is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposure, sizeof(exposure));

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

int main(void)
{
    cout << "Hello OpenCV and IDS ueye on Linux!" << endl;

    // configure cameras
    configureCamera(hCamRight, imgMemRight, memIdRight);
    // set the gain values for manual white balance
    is_SetHardwareGain(hCamRight, masterGain + 1, redGain, greenGain, blueGain);
    configureCamera(hCamLeft, imgMemLeft, memIdLeft);
    // set the exposer time in msec
    exposure -= 1.0;
    is_Exposure(hCamLeft, IS_EXPOSURE_CMD_SET_EXPOSURE,
                        (void*)&exposure, sizeof(exposure));

    // create the image matrices and display windows
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

    createButton("Threshold", on_threshold_click, NULL, CV_CHECKBOX, 0);
    createButton("Crosshairs", on_crosshairs_click, NULL, CV_CHECKBOX, 0);
    createTrackbar("Threshold", "", &intThresholdVal, 255,
                   on_threshold_change, NULL);

    // link camera image buffers to OpenCV matrices
    is_GetImageMem(hCamRight, &pMemVoidRight);    // get pointer to the image buffer
    is_GetImageMem(hCamLeft, &pMemVoidLeft);    // get pointer to the image buffer
    rightImageMat.data = (uchar *)pMemVoidRight;   // assign the pointer to Matrix
    leftImageMat.data = (uchar *)pMemVoidLeft;   // assign the pointer to Matrix

    // start acquisition loop
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
            warpAffine(rightImageMat, rightImageMat, M, rightImageMat.size());
            //
            // convert to binary threshold and find box corners...
            //
            if (!isCornersFound) {
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

                // Normalizing
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
                int loopCount = 0;
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

                isCornersFound = true;
            }
            //
            // apply threshold if selected...
            //
            if (isThreshold) {
                cvtColor(rightImageMat, rightGrayMat, COLOR_BGR2GRAY);
                threshold(rightGrayMat, rightGrayMat, thresholdVal, 255, THRESH_BINARY);
                cvtColor(leftImageMat, leftGrayMat, COLOR_BGR2GRAY);
                threshold(leftGrayMat, leftGrayMat, thresholdVal, 255, THRESH_BINARY);
                //
                // draw cross hairs...
                //
                if (isCrosshairs) {
                    line(rightGrayMat, ptLCtr, ptRCtr, white, lineWidth);
                    line(rightGrayMat, ptLMinus, ptRMinus, white, lineWidth);
                    line(rightGrayMat, ptCTop, ptCBot, white, lineWidth);
                    line(leftGrayMat, ptLCtr, ptRCtr, white, lineWidth);
                    line(leftGrayMat, ptLMinus, ptRMinus, white, lineWidth);
                    line(leftGrayMat, ptCTop, ptCBot, white, lineWidth);
                }
                imshow(right_image_win, rightGrayMat);
                imshow(left_image_win, leftGrayMat);
            }
            else {
                //
                // draw cross hairs...
                //
                if (isCrosshairs) {
                    line(rightImageMat, ptLCtr, ptRCtr, green, lineWidth);
                    line(rightImageMat, ptCTop, ptCBot, green, lineWidth);
                    line(leftImageMat, ptLCtr, ptRCtr, green, lineWidth);
                    line(leftImageMat, ptCTop, ptCBot, green, lineWidth);
                }
                //
                // Drawing a circle around corners
                //
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
                // find centroid
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
                //
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
