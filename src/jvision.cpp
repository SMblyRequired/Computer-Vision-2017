/*
 * jvision.cpp
 *  Class for image processing.
 *  Created on: Feb 10, 2017
 *      Author: josh ferrara
 */

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class JVision {
public:
    enum LOCK_TYPE {
        NO_LOCK,
        GEAR,
        BOILER
    };

    static string lockTypeToString(JVision::LOCK_TYPE cur) {
        if (cur == NO_LOCK) return "UNLOCKED";
        return (cur == BOILER ? "BOILER" : "GEAR");
    }

    JVision(string _networkAddress) : networkAddress(_networkAddress), curCapMethod(CAP_TYPE::NETWORK) {
        throw "Networked cameras not yet supported";
    }

    JVision(int _cameraNum) : usbCameraNum(_cameraNum), curCapMethod(CAP_TYPE::USB) {
        if (!usbCamera.open(usbCameraNum)) {
            throw "Could not open camera for read";
        }

        usbCamera.read(curRawFrame)

        initialized = true;
    }

    void run() {                    // Runs one iteration of the image processing algorithm
        if (!initialized) return;
        
        readFrame();

    }

    double getSolution() {          // Returns current calculated solution
        return solution;
    }

    double getFrameRate() {         // Returns current frame rate
        return cFrameRate;          
    }

    bool lockAcquired() {           // Returns true if target lock is acquired
        return (currentLock != LOCK_TYPE::NO_LOCK);
    }
private:
    void readFrame() {              // Reads a single frame from a video source
        if (curCapMethod == CAP_TYPE::USB) {
            if(!usbCamera.read(curRawFrame)) {
                throw "Error reading frame from camera.";
            }
        }
    }

    cv::Mat curRawFrame;            // Raw frame from the video source

    enum CAP_TYPE {                 // Capture type
        USB,
        NETWORK
    };

    bool initialized = false;       // Are we ready to process images?

    double hslHue[2] = {60, 75};    // Hue range
    double hslSat[2] = {240, 255};  // Saturation range
    double hslLum[2] = {80, 150};   // Luminescence range

    double cFrameRate;              // Current framerate/processing speed

    int usbCameraNum = -1;          // USB camera number
    cv::VideoCapture usbCamera;     // USB camera object

    string networkAddress;          // Networked camera address

    JVision::CAP_TYPE curCapMethod = CAP_TYPE::USB;

    double solution;                // Current solution.
    JVision::LOCK_TYPE currentLock = LOCK_TYPE::NO_LOCK;
};