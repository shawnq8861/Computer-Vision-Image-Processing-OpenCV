#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <uEye.h>

using namespace std;
using namespace cv;

// camera configuration parameters
const int img_width = 2560;
const int img_height = 1920;
int img_bpp = 24;
IS_RECT imageRect;
HIDS hCam = 1;
int masterGain = 0;
int redGain = 5;
int greenGain = 1;
int blueGain = 15;
double exposure = 98.0;
void *pMemVoid = NULL;         // pointer to where the image is stored

// openCV variables
const string &image_win = "Camera 1";
Mat rawImageMat;        // the raw image data from camera
Mat grayImageMat;

// other global variables
Point ptLCtr = Point(0, img_height/2);
Point ptRCtr = Point(img_width, img_height/2);
Point ptCTop = Point(img_width/2, 0);
Point ptCTopMinus = Point(img_width/2 - 100, 0);
Point ptCTopPlus = Point(img_width/2 + 100, 0);
Point ptCBot = Point(img_width/2, img_height);
Point ptCBotMinus = Point(img_width/2 - 100, img_height);
Point ptCBotPlus = Point(img_width/2 + 100, img_height);
double textFontSize = 2.2;
int lineWidth = 3;
double thresholdVal = 100;
bool isThreshold = false;
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

int main(void)
{
    cout << "Hello OpenCV and IDS ueye on Linux!" << endl;

    // configure camera
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

    double disable = 0;

    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &disable, 0);
    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &disable, 0);

    // set frame rate
    double newFPS = 0.0;
    double FPS = 10.0;
    is_SetFrameRate (hCam, FPS, &newFPS);
    cout << "frame rate set to " << newFPS << endl;

    is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &disable, 0);

    is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE,
                            (void*)&exposure, sizeof(exposure));

    is_SetHardwareGain(hCam, masterGain, redGain, greenGain, blueGain);

    // disable IR color correction
    double factors = 0.0;
    is_SetColorCorrection(hCam, IS_CCOR_DISABLE, &factors);

    // enable auto blacklevel
    uint8_t nMode = IS_AUTO_BLACKLEVEL_ON;
    is_Blacklevel(hCam, IS_BLACKLEVEL_CMD_SET_MODE, (void*)&nMode , sizeof(nMode ));

    // set auto blacl level offset
    uint8_t nOffset = 127;
    is_Blacklevel(hCam, IS_BLACKLEVEL_CMD_SET_OFFSET, (void*)&nOffset, sizeof(nOffset));

    // create the image matrices
    rawImageMat = Mat(Size(img_width, img_height), CV_8UC3);
    grayImageMat = rawImageMat;
    namedWindow(image_win, WINDOW_NORMAL | WINDOW_KEEPRATIO);

    createButton("Threshold", on_threshold_click, NULL, CV_CHECKBOX, 0);

    // link camera image to OpenCV matrix
    is_GetImageMem(hCam, &pMemVoid);    // get pointer to the image buffer
    rawImageMat.data = (uchar *)pMemVoid;   // assign the pointer to Matrix

    // start acquisition loop
    int waitRet = -1;
    while(1) {
        if (is_FreezeVideo(hCam, IS_WAIT) == IS_SUCCESS){
            if (isThreshold) {
                // convert to grayscale
                cvtColor(rawImageMat, grayImageMat, COLOR_BGR2GRAY);

                // apply histogram equalization
                equalizeHist(grayImageMat, grayImageMat);

                // apply binary threshold to image in place
                threshold(grayImageMat, grayImageMat, thresholdVal,
                      255, THRESH_BINARY);

                //
                // draw cross hairs...
                //
                line(grayImageMat, ptLCtr, ptRCtr, gray, lineWidth);
                line(grayImageMat, ptCTop, ptCBot, gray, lineWidth);
                line(grayImageMat, ptCTopMinus, ptCBotMinus, gray, lineWidth);
                line(grayImageMat, ptCTopPlus, ptCBotPlus, gray, lineWidth);

                imshow(image_win, grayImageMat);
            }
            else {
                //
                // draw cross hairs...
                //
                line(rawImageMat, ptLCtr, ptRCtr, green, lineWidth);
                line(rawImageMat, ptCTop, ptCBot, green, lineWidth);
                line(rawImageMat, ptCTopMinus, ptCBotMinus, red, lineWidth);
                line(rawImageMat, ptCTopPlus, ptCBotPlus, blue, lineWidth);

                imshow(image_win, rawImageMat);
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
    is_FreeImageMem(hCam, imgMem, memId);
    is_ExitCamera(hCam);

    return 0;
}
