#include <iostream>
#include <vector>
#include <fstream>
#include <windows.h>
#include <time.h>
#include <math.h>
#include "uEye.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "gclib.h"
#include "gclibo.h"

using namespace std;
using namespace cv;

// camera configuration parameters
const int img_width = 4912;
const int img_height = 3684;
//const int img_width = 2560;
//const int img_height = 1920;
int img_bpp = 24;
IS_RECT imageRect;

// camera calibration parameters
Mat intrinsicMatrix(3, 3, CV_64F);
Mat distortionCoeffs(5, 1, CV_64F);
Mat map1, map2;
vector<Mat> rvecs, tvecs;
vector< vector<Point3f> > objectPoints;
vector< vector<Point2f> > imagePoints;
vector<Point3f> objectCorners;
vector<Point2f> imageCorners;
const int colCount = 9;
const int rowCount = 6;
const int numCalImages = 10;
int calImageCount = 0;
const double squareSize = 31.75;  // size of the square on the chess board
double X = 0.0f;    // horizontal world coordinate
double Y = 0.0f;    // vertical world coordinate

// other parameters
double exposure = 17.0;
double maxBinaryVal = 255.0;
double threshholdVal = 200.0;
double textFontSize = 2.2;
int lineWidth = 3;
int masterGain = 1;
int redGain = 17;
int greenGain = 1;
int blueGain = 25;
int userDummyVal = -1;
int userPlaceHolder = 0;
int int_exposure = (int)exposure;
int int_thresholdVal = (int)threshholdVal;
int maxAdaptThreshVal = 255;
int tbBlockSize = 3;
int blockSize = 3;
int C = 10;
int mouseX = 0;
int mouseY = 0;
int rectX1 = 0;
int rectY1 = 0;
int rectX2 = 0;
int rectY2 = 0;
int pixelCount = 0;
int clickCount = 0;

boolean isGrayscale = false;
boolean isHistoEqualize = false;
boolean isThreshold = false;
boolean isAdaptThresh = false;
boolean isMarker = false;
boolean isDrawRectangle = false;
boolean drawRectangle = false;
boolean isFindContours = false;
boolean isSave = false;
boolean isSavePixels = false;
boolean isCalibrate = false;
boolean isCreateCalSamples = false;
boolean isQuit = false;
boolean isUndistort = false;
boolean isUndistorted = false;
boolean isFindChessboardCorners = false;
boolean isHarrisCorners = false;

string pixelStr = "pixel value = ";
const string &image_win = "Camera 1";
const double pi = 3.1415926;

HIDS hCam = 0;

Mat rawImageMat;        // the raw image data from camera
Mat modImageMat;        // the modified or transformed image data
Mat undistortImageMat;
Mat cornerImg;          // calibration images

vector<KeyPoint> keyPoints;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

// motion control variables
char buff[1024]; // Galil controller message buffer
GCon gConn = 0; // name of unique connection identifier
int testCount = 0;
const double doubleScaleHoriz = 20.997; // conversion from microns to steps orizontal
int pos_positionA = 0;  // slider value
int pos_stepsA = 0;     // step count from slider value
int neg_positionA = 0;  // slider value
int neg_stepsA;         // step count from slider value
int positionA = 0;      // current set position for A axis
int prevPositionA = 0;  // old set position for A axis
const double doubleScaleVert = 10.499; // conversion from microns to steps vertical
int pos_positionE = 0;  // slider value
int pos_stepsE = 0;     // step count from slider value
int neg_positionE = 0;  // initial slider value
int neg_stepsE;         // step count from slider value
int positionE = 0;      // current set position for E axis
int prevPositionE = 0;  // old set position for E axis

HANDLE positionAMutex;
HANDLE positionEMutex;

/*****************************************************************************
 * A basic thread function
 *
*****************************************************************************/
DWORD WINAPI testThread(LPVOID lpParameter)
{
    unsigned int& myCounter = *((unsigned int*)lpParameter);
    while(testCount < 0xFFFFFFFF && !isQuit){
        ++testCount;
        cout << "count = " << testCount << endl;
        Sleep(1000);
    }
    return 0;
}

// call to check return code from most gclib calls
void check(GReturn rc)
{
    if (rc != G_NO_ERROR)
    {
        cout << "ERROR: " << rc << endl;
        if (gConn > 0) {
            GClose(gConn);
        }
        exit (rc);
    }
}

boolean positionAChanged(void)
{
    // get the mutex
    WaitForSingleObject(positionAMutex, INFINITE);
    boolean changed = false;
    if (positionA != prevPositionA) {
        changed = true;
    }
    // release the mutex
    ReleaseMutex(positionAMutex);
    return changed;
}

/*****************************************************************************
 * Thread function for motion control of linear axis A
 *
*****************************************************************************/
DWORD WINAPI motionThreadA(LPVOID lpParameter)
{
    while(!isQuit){
        if (positionAChanged()) {
            // connect directly to COM3
            GOpen("COM3 --baud 115200 --subscribe ALL --direct", &gConn);

            // report position
            check(GCommand(gConn, "RP", buff, sizeof(buff), 0));
            cout << "RP response: " << buff << endl;

            // convert int distance to string and build move command
            stringstream posSS;

            // get the mutex
            WaitForSingleObject(positionAMutex, INFINITE);

            // build the command string
            double doubleStepsA = doubleScaleHoriz * (double)positionA;
            pos_stepsA = (int)doubleStepsA;
            cout << "axis A step count = " << pos_stepsA << endl;
            posSS << pos_stepsA;
            string posStr;
            posSS >> posStr;
            string command = "PAA= ";
            command.append(posStr);
            GCStringIn gComm = command.c_str();

            // execute move command
            check(GCommand(gConn, gComm, buff, sizeof(buff), 0));

            // begin move
            check(GCommand(gConn, "BG A", buff, sizeof(buff), 0));

            // wait for move complete
            GMotionComplete(gConn, "A");

            // update previous position to new position
            prevPositionA = positionA;

            // report position
            check(GCommand(gConn, "RP", buff, sizeof(buff), 0));
            cout << "RP response: " << buff << endl;

            // release the mutex
            ReleaseMutex(positionAMutex);

            if (gConn) {
                GClose(gConn); // always close the connection when finished
            }
        }
        else {
            Sleep(100);
        }
    }
    return 0;
}

void on_positionA_plus(int posAVal, void *userVal)
{
    // get the mutex
    WaitForSingleObject(positionAMutex, INFINITE);
    // save previous position
    prevPositionA = positionA;
    // update new position
    positionA = posAVal;
    // release the mutex
    ReleaseMutex(positionAMutex);
    stringstream posASS;
    posASS << positionA;
    string posAValStr;
    posASS >> posAValStr;
    string posAStr = "new positive A position value = ";
    posAStr.append(posAValStr);
    displayOverlay(image_win, posAStr, 1000);
}

void on_positionA_neg(int negAVal, void *userVal)
{
    // get the mutex
    WaitForSingleObject(positionAMutex, INFINITE);
    // save previous position
    prevPositionA = positionA;
    // update new position
    positionA = -negAVal;
    // release the mutex
    ReleaseMutex(positionAMutex);
    stringstream negASS;
    negASS << positionA;
    string negAValStr;
    negASS >> negAValStr;
    string negAStr = "new negative A position value = ";
    negAStr.append(negAValStr);
    displayOverlay(image_win, negAStr, 1000);
}

boolean positionEChanged(void)
{
    // get the mutex
    WaitForSingleObject(positionEMutex, INFINITE);
    boolean changed = false;
    if (positionE != prevPositionE) {
        changed = true;
    }
    // release the mutex
    ReleaseMutex(positionEMutex);
    return changed;
}

/*****************************************************************************
 * Thread function for motion control of linear axis E
 *
*****************************************************************************/
DWORD WINAPI motionThreadE(LPVOID lpParameter)
{
    while(!isQuit){
        if (positionEChanged()) {
            // connect directly to COM3
            GOpen("COM3 --baud 115200 --subscribe ALL --direct", &gConn);

            // report position
            check(GCommand(gConn, "RP", buff, sizeof(buff), 0));
            cout << "RP response: " << buff << endl;

            // convert int distance to string and build move command
            stringstream posSS;

            // get the mutex
            WaitForSingleObject(positionEMutex, INFINITE);

            // build the command string
            double doubleStepsE = doubleScaleVert * (double)positionE;
            pos_stepsE = (int)doubleStepsE;
            posSS << pos_stepsE;
            string posStr;
            posSS >> posStr;
            string command = "PAE= ";
            command.append(posStr);
            GCStringIn gComm = command.c_str();

            // execute move command
            check(GCommand(gConn, gComm, buff, sizeof(buff), 0));

            // begin move
            check(GCommand(gConn, "BG E", buff, sizeof(buff), 0));

            // wait for move complete
            GMotionComplete(gConn, "E");

            // update previous position to new position
            prevPositionE = positionE;

            // report position
            check(GCommand(gConn, "RP", buff, sizeof(buff), 0));
            cout << "RP response: " << buff << endl;

            // release the mutex
            ReleaseMutex(positionEMutex);

            if (gConn) {
                GClose(gConn); // always close the connection when finished
            }
        }
        else {
            Sleep(100);
        }
    }
    return 0;
}

void on_positionE_plus(int posEVal, void *userVal)
{
    // get the mutex
    WaitForSingleObject(positionEMutex, INFINITE);
    // save previous position
    prevPositionE = positionE;
    // update new position
    positionE = posEVal;
    // release the mutex
    ReleaseMutex(positionEMutex);
    stringstream posESS;
    posESS << positionE;
    string posEValStr;
    posESS >> posEValStr;
    string posEStr = "new positive E position value = ";
    posEStr.append(posEValStr);
    displayOverlay(image_win, posEStr, 1000);
}

void on_positionE_neg(int negEVal, void *userVal)
{
    // get the mutex
    WaitForSingleObject(positionEMutex, INFINITE);
    // save previous position
    prevPositionE = positionE;
    // update new position
    positionE = -negEVal;
    // release the mutex
    ReleaseMutex(positionEMutex);
    stringstream negESS;
    negESS << positionE;
    string negEValStr;
    negESS >> negEValStr;
    string negEStr = "new negative E position value = ";
    negEStr.append(negEValStr);
    displayOverlay(image_win, negEStr, 1000);
}

void findCentroid(vector<Point> contour, Point& center)
{
    int xMin = 10000;
    int xMax = -1;
    int yMin = 10000;
    int yMax = -1;
    vector<Point>::iterator pointIt;
    for (pointIt = contour.begin(); pointIt != contour.end(); ++pointIt) {
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

void onMouseAction(int event, int x, int y, int flags, void*)
{
    if (isDrawRectangle) {
        if (event == EVENT_LBUTTONDOWN) {
            drawRectangle = false;
            isMarker = false;
            rectX1 = x;
            rectY1 = y;
        }
        else if (event == EVENT_LBUTTONUP) {
            isMarker = false;
            rectX2 = x;
            rectY2 = y;
            drawRectangle = true;
        }
    }
    else {
        if (event == EVENT_LBUTTONDOWN) {
            isMarker = false;
        }
        else if (event == EVENT_LBUTTONUP) {
            mouseX = x;
            mouseY = y;
            isMarker = true;
            ++clickCount;
            if (clickCount == 3) {
                clickCount = 1;
            }
            cout << "click count = " << clickCount << endl;
        }
        else if (event == EVENT_MBUTTONUP) {
            isMarker = false;
        }
    }
}

void on_rectangle_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isDrawRectangle = true;
    }
    else if (buttonVal == 0) {
        isDrawRectangle = false;
    }
}

void on_contours_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isFindContours = true;
    }
    else if (buttonVal == 0) {
        isFindContours = false;
    }
}

void on_grayscale_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isGrayscale = true;
    }
    else if (buttonVal == 0) {
        isGrayscale = false;
    }
}

void on_histoEqual_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isHistoEqualize = true;
    }
    else if (buttonVal == 0) {
        isHistoEqualize = false;
    }
}

void on_threshold_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isThreshold = true;
    }
    else if (buttonVal == 0) {
        isThreshold = false;
    }
}

void on_adapt_thresh_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isAdaptThresh = true;
    }
    else if (buttonVal == 0) {
        isAdaptThresh = false;
    }
}

void on_threshold_change(int threshVal, void *userVal)
{
    int_thresholdVal = threshVal;
    stringstream threshSS;
    threshSS << int_thresholdVal;
    string threshValStr;
    threshSS >> threshValStr;
    string threshStr = "new threshold value = ";
    threshStr.append(threshValStr);
    displayOverlay(image_win, threshStr, 1000);
}

void on_block_size_change(int blockSizeVal, void *userVal)
{
    if (blockSizeVal < 3) {
        blockSizeVal = 3;
    }
    // if it is even, add 1 to it
    if (blockSizeVal % 2 == 0) {
        blockSizeVal += 1;
    }
    blockSize = blockSizeVal;
    stringstream blockSizeSS;
    blockSizeSS << blockSize;
    string blockSizeValStr;
    blockSizeSS >> blockSizeValStr;
    string blockSizeStr = "new block size value = ";
    blockSizeStr.append(blockSizeValStr);
    displayOverlay(image_win, blockSizeStr, 1000);
}

void on_C_change(int CVal, void *userVal)
{
    C = CVal;
    stringstream CSS;
    CSS << CVal;
    string CValStr;
    CSS >> CValStr;
    string CStr = "new C value = ";
    CStr.append(CValStr);
    displayOverlay(image_win, CStr, 1000);
}

void on_exposure_change(int exposVal, void *userVal)
{
    exposure = (double)exposVal;
    is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE,
                            (void*)&exposure, sizeof(exposure));
    stringstream exposSS;
    exposSS << exposVal;
    string exposValStr;
    exposSS >> exposValStr;
    string exposStr = "new exposure value = ";
    exposStr.append(exposValStr);
    displayOverlay(image_win, exposStr, 1000);
}

void on_gain_change(int gainVal, void *userVal)
{
    is_SetHardwareGain(hCam, gainVal, redGain, greenGain, blueGain);
    stringstream gainSS;
    gainSS << gainVal;
    string gainValStr;
    gainSS >> gainValStr;
    string gainStr = "new gain value = ";
    gainStr.append(gainValStr);
    displayOverlay(image_win, gainStr, 1000);
}

void on_red_change(int redVal, void *userVal)
{
    is_SetHardwareGain(hCam, masterGain, redVal, greenGain, blueGain);
    stringstream redSS;
    redSS << redVal;
    string redValStr;
    redSS >> redValStr;
    string redStr = "new red value = ";
    redStr.append(redValStr);
    displayOverlay(image_win, redStr, 1000);
}

void on_green_change(int greenVal, void *userVal)
{
    is_SetHardwareGain(hCam, masterGain, redGain, greenVal, blueGain);
    stringstream greenSS;
    greenSS << greenVal;
    string greenValStr;
    greenSS >> greenValStr;
    string greenStr = "new green value = ";
    greenStr.append(greenValStr);
    displayOverlay(image_win, greenStr, 1000);
}

void on_blue_change(int blueVal, void *userVal)
{
    is_SetHardwareGain(hCam, masterGain, redGain, greenGain, blueVal);
    stringstream blueSS;
    blueSS << blueVal;
    string blueValStr;
    blueSS >> blueValStr;
    string blueStr = "new blue value = ";
    blueStr.append(blueValStr);
    displayOverlay(image_win, blueStr, 1000);
}

void on_save_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
    }
    else if (buttonVal == 0) {
        cout << "save button clicked value = 0" << endl;
        isSave = true;
    }
}

void on_calibrate_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
    }
    else if (buttonVal == 0) {
        cout << "calibrate button clicked value = 0" << endl;
        isCalibrate = true;
    }
}

void on_quit_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
    }
    else if (buttonVal == 0) {
        cout << "Quit button clicked value = 0" << endl;
        isQuit = true;
    }
}

void on_undistort_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isUndistort = true;
    }
    else if (buttonVal == 0) {
        isUndistort = false;
        isUndistorted = false;
    }
}

void on_chessboard_corners_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isFindChessboardCorners = true;
        cout << "corners checked..." << endl;
    }
    else if (buttonVal == 0) {
        isFindChessboardCorners = false;
    }
}

void on_harris_corners_click(int buttonVal, void *userVal)
{
    if (buttonVal == 1) {
        isHarrisCorners = true;
        cout << "corners checked..." << endl;
    }
    else if (buttonVal == 0) {
        isHarrisCorners = false;
    }
}


int main()
{
    char *imgMem = NULL;
    int memId;
    if (is_InitCamera(&hCam, NULL) != IS_SUCCESS){
        cout << "could not get camera handle..." << endl;
        return 0;
    }

    // memory initialization
    is_AllocImageMem(hCam, img_width, img_height, img_bpp, &imgMem, &memId);

    // set memory active
    is_SetImageMem(hCam, imgMem, memId);

    imageRect.s32X = 0;
    imageRect.s32Y = 0;
    imageRect.s32Width = img_width;
    imageRect.s32Height = img_height;

    is_SetDisplayMode(hCam, IS_SET_DM_DIB);

    // Set the AOI with the correct size
    is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&imageRect, sizeof(imageRect));

    double enable = 1;
    double disable = 0;

    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0);

    // set frame rate
    double newFPS = 1.0;
    double FPS = 1.0;
    is_SetFrameRate (hCam, FPS, &newFPS);

    // set gamma correction to 1.6
    int nGamma = 160;
    is_Gamma(hCam, IS_GAMMA_CMD_SET, (void*) &nGamma, sizeof(nGamma));

    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &disable, 0);

    is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE,
                            (void*)&exposure, sizeof(exposure));

    //is_SetGainBoost (hCam, IS_SET_GAINBOOST_ON);

    is_SetHardwareGain(hCam, masterGain, redGain, greenGain, blueGain);

    // create the image matrices
    rawImageMat = Mat(Size(img_width, img_height), CV_8UC3);

    void *pMemVoid = NULL;         // pointer to where the image is stored

    modImageMat = Mat(rawImageMat); // create copy of raw image to use in analysis

    namedWindow(image_win, WINDOW_NORMAL | WINDOW_KEEPRATIO);

    // create check boxes
    createButton("Rectangle", on_rectangle_click, NULL, CV_CHECKBOX, 0);
    createButton("Contours", on_contours_click, NULL, CV_CHECKBOX, 0);
    createButton("Grayscale", on_grayscale_click, NULL, CV_CHECKBOX, 0);
    createButton("HistoEqualize", on_histoEqual_click, NULL, CV_CHECKBOX, 0);
    createButton("Threshold", on_threshold_click, NULL, CV_CHECKBOX, 0);
    createButton("Adaptive Threshold", on_adapt_thresh_click, NULL,
                 CV_CHECKBOX, 0);
    createButton("Undistort", on_undistort_click, NULL,
                 CV_CHECKBOX, 0);
    createButton("HarrisCorn", on_harris_corners_click, NULL, CV_CHECKBOX, 0);
    createButton("Corners", on_chessboard_corners_click, NULL, CV_CHECKBOX, 0);

    // create push buttons

    createButton("Save Image", on_save_click, NULL, CV_PUSH_BUTTON, 0);
    createButton("Calibrate", on_calibrate_click, NULL, CV_PUSH_BUTTON, 0);
    createButton("Quit", on_quit_click, NULL, CV_PUSH_BUTTON, 0);

    // create trackbars
    createTrackbar("shutter", "", &int_exposure, 100,
                   on_exposure_change, NULL);
    createTrackbar("gain", "", &masterGain, 100, on_gain_change, NULL);
    createTrackbar("red", "", &redGain, 100, on_red_change, NULL);
    createTrackbar("green", "", &greenGain, 100, on_green_change, NULL);
    createTrackbar("blue", "", &blueGain, 100, on_blue_change, NULL);
    createTrackbar("Threshold", "", &int_thresholdVal, 255,
                   on_threshold_change, NULL);
    createTrackbar("Block Size", "", &tbBlockSize, 30,
                   on_block_size_change, NULL);
    createTrackbar("C", "", &C, 30, on_C_change, NULL);
    createTrackbar("A+ Position", image_win, &pos_positionA, 95000,
                   on_positionA_plus, NULL);
    createTrackbar("A- Position", image_win, &neg_positionA, 95000,
                   on_positionA_neg, NULL);
    createTrackbar("E+ Position", image_win, &pos_positionE, 8000,
                   on_positionE_plus, NULL);
    createTrackbar("E- Position", image_win, &neg_positionE, 8000,
                   on_positionE_neg, NULL);

    setMouseCallback(image_win, onMouseAction, NULL);

    Point ptLCtr = Point(0, img_height/2);
    Point ptRCtr = Point(img_width, img_height/2);
    Point ptCTop = Point(img_width/2, 0);
    Point ptCBot = Point(img_width/2, img_height);
    Point ptLeft, ptRight, ptTop, ptBot, ptRect1, ptRect2;
    Point ptPixRectL, ptPixRectR;
    Point angle1, angle2;

    Scalar blue = Scalar(255, 0, 0);
    Scalar green = Scalar(0, 255, 0);
    Scalar red = Scalar(0, 0, 255);
    Scalar white = Scalar(255, 255, 255);
    Scalar gray = Scalar(127, 127, 127);

    is_GetImageMem(hCam, &pMemVoid);    // fill the buffer with data
    rawImageMat.data = (uchar *)pMemVoid;   // assign pointer to Matrix
    modImageMat = rawImageMat;      // initialize modified image to raw image

    //
    // create the mutex to protect access to position data
    //
    positionAMutex = CreateMutexA(0, 0, 0);

    //
    // create and start the test thread
    //
    //DWORD testThreadID;
    //HANDLE testThreadHandle = CreateThread(
                //0, 0, testThread, 0, 0, &testThreadID);
    //
    // create and start the motion thread for axis A
    //
    DWORD motionThreadAID;
    HANDLE motionThreadHandleA = CreateThread(
                0, 0, motionThreadA, 0, 0, &motionThreadAID);

    //
    // create and start the motion thread for axis A
    //
    DWORD motionThreadEID;
    HANDLE motionThreadHandleE = CreateThread(
                0, 0, motionThreadE, 0, 0, &motionThreadEID);

    int waitRet = -1;
    while(1) {
        if (is_FreezeVideo(hCam, IS_WAIT) == IS_SUCCESS){
            if (isGrayscale && !isHistoEqualize && !isThreshold &&
                    !isAdaptThresh && !isFindContours) {
                cvtColor(rawImageMat, modImageMat, COLOR_BGR2GRAY);
            }
            else if (isHistoEqualize && !isThreshold && !isAdaptThresh
                     && !isFindContours) {
                // always convert to grayscale first
                cvtColor(rawImageMat, modImageMat, COLOR_BGR2GRAY);
                // apply histogram equalization to image in place
                equalizeHist(modImageMat, modImageMat);
            }
            else if (isThreshold && !isAdaptThresh && !isFindContours) {
                // always convert to grayscale first
                cvtColor(rawImageMat, modImageMat, COLOR_BGR2GRAY);
                // apply histogram equalization in place if selected
                if (isHistoEqualize) {
                    equalizeHist(modImageMat, modImageMat);
                }
                // apply simple threshold to image in place
                threshholdVal = (double)int_thresholdVal;
                threshold(modImageMat, modImageMat, threshholdVal,
                          maxBinaryVal, THRESH_BINARY);
            }
            else if (isAdaptThresh && !isThreshold && !isFindContours) {
                // always convert to grayscale first
                cvtColor(rawImageMat, modImageMat, COLOR_BGR2GRAY);
                // apply histogram equalization in place if selected
                if (isHistoEqualize) {
                    equalizeHist(modImageMat, modImageMat);
                }
                adaptiveThreshold(modImageMat, modImageMat, 255,
                                  ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY,
                                  blockSize, C);
            }
            else if(isFindContours && !isThreshold && !isAdaptThresh) {
                // always convert to grayscale first
                cvtColor(rawImageMat, modImageMat, COLOR_BGR2GRAY);
                Canny(modImageMat, modImageMat, 100, int_thresholdVal, 3);
                dilate(modImageMat, modImageMat, Mat(), Point(-1,-1));
                findContours( modImageMat, contours, hierarchy, CV_RETR_TREE,
                              CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
                modImageMat = rawImageMat;
            }
            else {
                modImageMat = rawImageMat;
            }

            //
            // draw center cross hairs...
            //
            line(modImageMat, ptLCtr, ptRCtr, green, lineWidth);
            line(modImageMat, ptCTop, ptCBot, green, lineWidth);

            //
            // draw the marker cross hair
            //
            if (isMarker) {
                ptLeft = Point(0, mouseY);
                ptRight = Point(img_width, mouseY);
                ptTop = Point(mouseX, 0);
                ptBot = Point(mouseX, img_height);
                line(modImageMat, ptLeft, ptRight, blue, lineWidth);
                line(modImageMat, ptTop, ptBot, blue, lineWidth);

                // calculate the horizontal and vertical center distances
                int centerDistMX = mouseX - img_width/2;
                int centerDistMY = mouseY -img_height/2;
                //
                // build up string to display
                //
                stringstream measureSSMX;
                measureSSMX << centerDistMX;
                string measValStrMX;
                measureSSMX >> measValStrMX;
                string measureStrMX = " horiz pixs";
                measValStrMX.append(measureStrMX);
                stringstream measureSSMY;
                measureSSMY << centerDistMY;
                string measValStrMY;
                measureSSMY >> measValStrMY;
                string measureStrMY = " vert pixs";
                measValStrMY.append(measureStrMY);
                string measAngleStr = " angle:  ";
                //
                // check mouse click count and calculate and draw angle
                //
                if (clickCount == 1) {
                    angle1.x = mouseX;
                    angle1.y = mouseY;
                }
                else if (clickCount == 2) {
                    angle2.x = mouseX;
                    angle2.y = mouseY;
                    double base = (double)angle2.x - (double)angle1.x;

                    double height = (double)angle1.y - (double)angle2.y;

                    double hypo = hypot(base, height);
                    double angle = 0;
                    if (base >= 0.0 && height >= 0.0) {
                        angle = asin(height/hypo) * 180.0/pi;
                    }
                    else if (base >= 0.0 && height < 0.0) {
                        angle = asin(height/hypo) * 180.0/pi;
                    }
                    else if (base < 0.0 && height >= 0.0) {
                        angle = 180 - asin(height/hypo) * 180.0/pi;
                    }
                    else if (base < 0.0 && height < 0.0) {
                        angle = -180 - asin(height/hypo) * 180.0/pi;
                    }
                    //
                    // build up string to display
                    //
                    stringstream angleSS;
                    angleSS << angle;
                    string angleStr;
                    angleSS >> angleStr;
                    measAngleStr.append(angleStr);
                }
                //
                // display the strings near the crosshair
                //
                if (centerDistMX > 0 && centerDistMY < 0) {
                    putText(modImageMat, measValStrMX,
                            Point(mouseX - (measureStrMX.length() * 50), mouseY + 60),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measValStrMY,
                            Point(mouseX - (measureStrMX.length() * 50), mouseY + 120),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measAngleStr,
                            Point(mouseX - (measureStrMX.length() * 60), mouseY + 180),
                            FONT_HERSHEY_SIMPLEX, textFontSize, white, lineWidth);
                }
                else if (centerDistMX < 0 && centerDistMY < 0){
                    putText(modImageMat, measValStrMX,
                            Point(mouseX, mouseY + 60),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measValStrMY,
                            Point(mouseX, mouseY + 120),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measAngleStr,
                            Point(mouseX, mouseY + 180),
                            FONT_HERSHEY_SIMPLEX, textFontSize, white, lineWidth);
                }
                else if (centerDistMX < 0 && centerDistMY > 0) {
                    putText(modImageMat, measValStrMX,
                            Point(mouseX, mouseY - 20),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measValStrMY,
                            Point(mouseX, mouseY - 80),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                    putText(modImageMat, measAngleStr,
                            Point(mouseX, mouseY - 160),
                            FONT_HERSHEY_SIMPLEX, textFontSize, white, lineWidth);
                }
                else if (centerDistMX > 0 && centerDistMY > 0) {
                putText(modImageMat, measValStrMX,
                        Point(mouseX  - (measureStrMX.length() * 50), mouseY - 20),
                        FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                putText(modImageMat, measValStrMY,
                        Point(mouseX  - (measureStrMX.length() * 50), mouseY - 80),
                        FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                putText(modImageMat, measAngleStr,
                        Point(mouseX  - (measureStrMX.length() * 60), mouseY - 160),
                        FONT_HERSHEY_SIMPLEX, textFontSize, white, lineWidth);
                }
            }

            //
            // Large if, else if, else follows to
            // draw a rectangle
            //
            // if...
            //
            if (isDrawRectangle) {
                if (drawRectangle) {
                    ptRect1 = Point(rectX1, rectY1);
                    ptRect2 = Point(rectX2, rectY2);
                    rectangle(modImageMat, Rect(ptRect1, ptRect2), red, lineWidth);
                    cout << "rectangle at "
                         << rectX1 << ", "
                         << rectY1 << " and "
                         << rectX2 << ", "
                         << rectY2 << endl;
                }
            }

            //
            // Large if, else if, else follows to
            // display the modified or transformed image...
            //
            // else if...
            //
            else if (isFindContours) {
                if (isMarker) {
                    isMarker = false;
                    modImageMat = rawImageMat;
                }
                double maxArea = -1.0;
                int maxContourIndex = -1;
                for( int i = 0; i< contours.size(); i++ ) {
                    double temp = contourArea(contours.at(i));
                    if (maxArea < temp) {
                        maxArea = temp;
                        maxContourIndex = i;
                    }
                }
                drawContours( modImageMat, contours, maxContourIndex,
                              blue, 2, 8, hierarchy, 0, Point() );
                Point center, ptCenterL, ptCenterR, ptCenterT, ptCenterB;
                findCentroid(contours.at(maxContourIndex), center);
                ptCenterL = Point(0, center.y);
                ptCenterR = Point(img_width, center.y);
                ptCenterT = Point(center.x, 0);
                ptCenterB = Point(center.x, img_height);
                line(modImageMat, ptCenterL, ptCenterR, blue, lineWidth);
                line(modImageMat, ptCenterT, ptCenterB, blue, lineWidth);

                // calculate the horizontal vertical center distances
                int centerDistX = center.x - img_width/2;
                int centerDistY = center.y - img_height/2;
                //
                // build up strings to display
                //
                stringstream measureSSX;
                measureSSX << centerDistX;
                string measValStrX;
                measureSSX >> measValStrX;
                string measureStrX = " horiz pixs";
                measValStrX.append(measureStrX);
                stringstream measureSSY;
                measureSSY << centerDistY;
                string measValStrY;
                measureSSY >> measValStrY;
                string measureStrY = " vert pixs";
                measValStrY.append(measureStrY);
                //
                // display the strings near the crosshair
                //
                if (centerDistX > 0) {
                    putText(modImageMat, measValStrX,
                            Point(center.x, center.y -25), FONT_HERSHEY_SIMPLEX,
                            textFontSize, red, lineWidth);
                }
                else {
                    putText(modImageMat, measValStrX,
                            Point(center.x, center.y -25), FONT_HERSHEY_SIMPLEX,
                            textFontSize, red, lineWidth);
                }
                if (centerDistY > 0) {
                    putText(modImageMat, measValStrY,
                            Point(center.x, center.y -60), FONT_HERSHEY_SIMPLEX,
                            textFontSize, red, lineWidth);
                }
                else {
                    putText(modImageMat, measValStrY,
                            Point(center.x, center.y +45),
                            FONT_HERSHEY_SIMPLEX, textFontSize, red, lineWidth);
                }
            }

            //
            // find and draw harris corners
            //
            //else if...
            //
            else if (isHarrisCorners) {
                Mat src_gray;
                Mat dst, dst_norm, dst_norm_scaled;
                dst = Mat::zeros( rawImageMat.size(), CV_32FC1 );

                // Detector parameters
                int blockSize = 8;
                int apertureSize = 17;
                double k = 0.08;

                // Detecting corners
                cvtColor(rawImageMat, src_gray, COLOR_BGR2GRAY);
                cout << "finding harris corners..." << endl;
                cornerHarris( src_gray, dst, blockSize,
                              apertureSize, k, BORDER_DEFAULT );

                // Normalizing
                normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
                convertScaleAbs( dst_norm, dst_norm_scaled );

                // Drawing a circle around corners
                cout << "drawing harris corners..." << endl;
                int cornerCount = 0;
                for( int j = 0; j < dst_norm.rows ; j++ ) {
                    for( int i = 0; i < dst_norm.cols; i++ ) {
                        if( (int) dst_norm.at<float>(j,i) > int_thresholdVal ) {
                            ++cornerCount;
                            circle( modImageMat, Point( i, j ), 35,  red, 2, 8, 0 );
                        }
                    }
                }
                cout << "found " << cornerCount << " corners" << endl;
            }

            //
            // find and draw chessboard corners before saving cal image
            //
            // else if...
            //
            else if (isFindChessboardCorners) {
                Mat src_gray;
                cvtColor(rawImageMat, src_gray, COLOR_BGR2GRAY);
                Size checkerSize(colCount, rowCount);
                cout << "trying to find chessboard corners for:" << endl
                     << "columns = " << colCount << endl
                     << "rows = " << rowCount << endl;
                boolean cornersFound = findChessboardCorners(src_gray,
                                                             checkerSize,
                                                             imageCorners,
                                                             CALIB_CB_ADAPTIVE_THRESH +
                                                             CALIB_CB_NORMALIZE_IMAGE +
                                                             CALIB_CB_FAST_CHECK);
                if (cornersFound) {
                    cout << "found " << imageCorners.size() << " chessboard corners" << endl;
                    cornerSubPix(src_gray, imageCorners, Size(11, 11), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                    drawChessboardCorners(modImageMat, checkerSize, Mat(imageCorners), cornersFound);
                }
                else {
                    cout << "could not find chessboard corners" << endl;
                }
                string cornerWinName("Corner Image");
                namedWindow(cornerWinName, WINDOW_NORMAL | WINDOW_KEEPRATIO);
                imshow(cornerWinName, modImageMat);
                cout << "done finding chessboard corners" << endl;
                isFindChessboardCorners = false;
            }
            //
            // display the final image...
            //
            // else...
            //
            else {
                imshow(image_win, modImageMat);
            }

            //
            // save an image to the hard drive
            //
            if (isSave) {
                if (isCreateCalSamples) {
                    //
                    // create calibration images until count complete
                    //
                    cout << "creating calibration image number " << calImageCount << endl;
                    string fileName("calImage");
                    string filePath("C:\\CalibrationImages\\");
                    filePath.append(fileName);
                    stringstream imCountSS;
                    string imCountStr;
                    imCountSS << calImageCount;
                    imCountSS >> imCountStr;
                    filePath.append(imCountStr);
                    filePath.append(".jpg");
                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(95);
                    try {
                        imwrite(filePath, modImageMat, compression_params);
                    }
                    catch (Exception& ex) {
                        fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
                        return 1;
                    }
                    ++calImageCount;
                    if (calImageCount > numCalImages) {
                        calImageCount = 0;
                        isCreateCalSamples = false;
                    }
                }
                else {
                    //
                    // normal single save
                    //
                    // ask if we want new sample images
                    //
                    cout << "New sample images? (Y/N) ";
                    char respChar;
                    cin >> respChar;
                    if (respChar == 'y' || respChar == 'Y') {
                        ++calImageCount;    // correct count for first image
                        cout << endl << "you have asked to create new sample images..." << endl;
                        isCreateCalSamples = true;
                    }
                    else {
                        string fileName;
                        cout << "enter file name: ";
                        cin >> fileName;
                        string filePath("C:\\SavedImages\\");
                        filePath.append(fileName);
                        filePath.append(".jpg");
                        vector<int> compression_params;
                        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                        compression_params.push_back(95);
                        try {
                            if (isUndistorted) {
                                imwrite(filePath, undistortImageMat, compression_params);
                                cout << "saving undistorted image to file" << endl;
                            }
                            else {
                                imwrite(filePath, modImageMat, compression_params);
                                cout << "saving distorted image to file" << endl;
                            }
                        }
                        catch (Exception& ex) {
                            fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
                            return 1;
                        }
                    }
                }
                isSave = false;
            }

            //
            // calibration code follows
            //
            if (isCalibrate) {
                // initialize the 3D object corner values
                for (int row = 0; row < rowCount; ++row) {
                    for (int col = 0; col < colCount; ++col) {
                        objectCorners.push_back(Point3f(X, Y, 0.0f));
                        cout << "X = " << X << ", Y = " << Y << endl;
                        Y += squareSize;
                    }
                    Y = 0.0f;
                    X += squareSize;
                }
                cout << "starting calibration loop" << endl;

                string baseFileName;
                cout << "enter base file name: ";
                cin >> baseFileName;
                string filePath("C:\\SavedImages\\");
                filePath.append(baseFileName);

                for (int i = 0; i < numCalImages; ++i) {
                    string fileName = filePath;

                    stringstream imCountSS;
                    string imCountStr;
                    imCountSS << i + 1;
                    imCountSS >> imCountStr;
                    fileName.append(imCountStr);
                    fileName.append(".jpg");

                    cout << "reading file: " << fileName << endl;
                    cornerImg = imread(fileName, IMREAD_COLOR);
                    cvtColor(cornerImg, cornerImg, COLOR_BGR2GRAY);
                    if (cornerImg.empty()) {
                        cout << "could not read image..." << endl;
                    }
                    else {
                        Size checkerSize(colCount, rowCount);
                        boolean cornersFound = findChessboardCorners(cornerImg, checkerSize, imageCorners,
                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
                        if (cornersFound) {
                            cornerSubPix(cornerImg, imageCorners, Size(11, 11), Size(-1, -1),
                                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                            cvtColor(cornerImg, cornerImg, COLOR_GRAY2BGR);
                            drawChessboardCorners(cornerImg, checkerSize, Mat(imageCorners), cornersFound);
                            string cornerWinName("Corner Image ");
                            cornerWinName.append(imCountStr);
                            namedWindow(cornerWinName, WINDOW_NORMAL | WINDOW_KEEPRATIO);
                            imshow(cornerWinName, cornerImg);
                            //
                            // save the image and object corners
                            //
                            objectPoints.push_back(objectCorners);
                            imagePoints.push_back(imageCorners);
                        }
                        else {
                            cout << "could not find corners for image " << imCountStr << endl;
                        }
                    }

                }

                //
                // calculate intrinsic matrix values and radial distortion coefficients
                //
                double calRetVal = calibrateCamera (objectPoints, imagePoints, modImageMat.size(),
                                                    intrinsicMatrix, distortionCoeffs, rvecs, tvecs);
                cout << "calibration return value = " << calRetVal << endl;
                cout << "instrinsic matrix values:" << endl << endl;
                cout << "fx = " << intrinsicMatrix.at<double>(0, 0) << endl;
                cout << intrinsicMatrix.at<double>(0, 1) << endl;
                cout << "u0 = " << intrinsicMatrix.at<double>(0, 2) << endl;
                cout << intrinsicMatrix.at<double>(1, 0) << endl;
                cout << "fy = " << intrinsicMatrix.at<double>(1, 1) << endl;
                cout << "v0 = " << intrinsicMatrix.at<double>(1, 2) << endl;
                cout << intrinsicMatrix.at<double>(2, 0) << endl;
                cout << intrinsicMatrix.at<double>(2, 1) << endl;
                cout << intrinsicMatrix.at<double>(2, 2) << endl;
                cout << "distortion matrix values:" << endl << endl;
                cout << "k1 = " << distortionCoeffs.at<double>(0) << endl;
                cout << "k2 = "<< distortionCoeffs.at<double>(1) << endl;
                cout << "p1 = "<< distortionCoeffs.at<double>(2) << endl;
                cout << "p2 = "<< distortionCoeffs.at<double>(3) << endl;
                cout << "k3 = "<< distortionCoeffs.at<double>(4) << endl;

                isCalibrate = false;
            }

            //
            // undistort the image
            //
            if (isUndistort) {
                if (!isUndistorted) {
                    cout << "calculating undistortion..." << endl;
                    initUndistortRectifyMap(intrinsicMatrix, distortionCoeffs, Mat(),
                                Mat(), modImageMat.size(), CV_32FC1, map1, map2);
                    isUndistorted = true;
                }
                remap(rawImageMat, undistortImageMat, map1, map2, INTER_LINEAR);
                namedWindow("Undistorted Image", WINDOW_NORMAL | WINDOW_KEEPRATIO);
                imshow("Undistorted Image", undistortImageMat);
            }


            //
            // check if escape key pressed to terminate...
            //
            waitRet = waitKey(2);
            if (waitRet == 27 || isQuit) {
                isQuit = true;
                break;
            }
        }
    }
    WaitForSingleObject(motionThreadHandleE, INFINITE);
    WaitForSingleObject(motionThreadHandleA, INFINITE);
    //WaitForSingleObject(testThreadHandle, INFINITE);
    CloseHandle(positionAMutex);
    CloseHandle(motionThreadHandleE);
    CloseHandle(motionThreadHandleA);
    //CloseHandle(testThreadHandle);
    destroyAllWindows();
    is_FreeImageMem(hCam, imgMem, memId);
    is_ExitCamera(hCam);

    return 0;
}

