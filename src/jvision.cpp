/*
 * jvision.cpp
 *  Class for image processing.
 *  Created on: Feb 10, 2017
 *      Author: josh ferrara
 *
 *  Notes: Ideally, I would like to remove the imshows from this file, and instead provide an interface
 *  for retrieving the output of a specific step within the CV process. The general idea would be to do
 *  the main processing in this file, and allow user interaction with the CV process from the main file
 */

// #define VISUALSTEPS

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ntcore.h>

#include "jcverr.cpp"
#include "utils.cpp"

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
            throw CameraUnavailable();
        }

        usbCamera.read(curRawFrame);

        finishInit();
    }

    void setHeadless(bool _headless) {
    	headless = _headless;
    }

    double run() {                    		// Runs one iteration of the image processing algorithm
        if (!initialized) throw std::runtime_error("Cannot run algorithm without initialization.");
        int64 start = cv::getTickCount();	// Storage for current time so that we can calculate frame rate
        
        readFrame();

#ifdef VISUALSTEPS
		cv::Mat hslOut2;
		cv::cvtColor(curRawFrame, hslOut2, cv::COLOR_BGR2HLS); // Convert from BGS to HLS

		cv::Vec3b colorAtZeroZero = hslOut2.at<cv::Vec3b>(cv::Point(curRawFrame.cols / 2, curRawFrame.rows / 2));
		cv::putText(hslOut2, "Color at " + to_string(curRawFrame.cols / 2) + ", " + to_string(curRawFrame.rows / 2) + " (HLS): {" + to_string(colorAtZeroZero[0]) + ", " + to_string(colorAtZeroZero[1]) + ", " + to_string(colorAtZeroZero[2]) + "}", cvPoint(0, 45), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(80, 255, 255));

		cv::line(hslOut2, cv::Point(curRawFrame.cols / 2, 0), cv::Point(curRawFrame.cols / 2, curRawFrame.rows), cv::Scalar(0, 255, 0), 1);
		cv::line(hslOut2, cv::Point(0, curRawFrame.rows / 2), cv::Point(curRawFrame.cols, curRawFrame.rows / 2), cv::Scalar(0, 255, 0), 1);

		cv::imshow("Webcam Unprocessed HSL - Cam #" + to_string(usbCameraNum), hslOut2);
#endif

		// HSL threshold for vision target filtering
		cv::Mat hslOut;
		cv::cvtColor(curRawFrame, hslOut, cv::COLOR_BGR2HLS); // Convert from BGS to HLS
		cv::inRange(hslOut, cv::Scalar(hslHue[0], hslLum[0], hslSat[0]), cv::Scalar(hslHue[1], hslLum[1], hslSat[1]), hslOut); // Copy all points within range
#ifdef VISUALSTEPS
		cv::Mat hslCpy;
		hslOut.copyTo(hslCpy);

		cv::putText(hslCpy, "HSL LBound {" + to_string(hslHue[0]) + ", " + to_string(hslSat[0]) + ", " + to_string(hslLum[0]) + "}", cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);
		cv::putText(hslCpy, "HSL UBound {" + to_string(hslHue[1]) + ", " + to_string(hslSat[1]) + ", " + to_string(hslLum[1]) + "}", cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::imshow("HSL filter - Cam #" + to_string(usbCameraNum), hslCpy);
#endif

		// Find contours
		std::vector<std::vector<cv::Point>> contoursUnfiltered;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(hslOut, contoursUnfiltered, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
#ifdef VISUALSTEPS
		cv::Mat conOut;
		curRawFrame.copyTo(conOut);

		if (contoursUnfiltered.size() > 0) {
			int i = 0;
			for (; i >= 0; i = hierarchy[i][0]) {
				cv::drawContours(conOut, contoursUnfiltered, i, cv::Scalar(0, 0, 255), 5);
			}
		}

		cv::putText(conOut, "Unfiltered contours found: " + to_string(contoursUnfiltered.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::imshow("Contours found - Cam #" + to_string(usbCameraNum), conOut);
#endif

		// Filter contours
		std::vector<std::vector<cv::Point>> contours;
		filterContours(contoursUnfiltered, contours);

		// Convex hulls
		std::vector<std::vector<cv::Point>> hulls(contours.size());
		for (size_t i = 0; i < contours.size(); i++) {
			cv::convexHull(cv::Mat((contours)[i]), hulls[i], false);
		}

#ifdef VISUALSTEPS
		cv::Mat hullOut;
		curRawFrame.copyTo(hullOut);

		if (hulls.size() > 0) {
			for (int i = 0; i < hulls.size(); i++) {
				cv::drawContours(hullOut, hulls, i, cv::Scalar(0, 0, 255), 5);

				cv::Rect conRect = cv::boundingRect(hulls[i]);
				cv::rectangle(hullOut, conRect, cv::Scalar(255, 255, 0), 2);
			}
		}

		cv::putText(hullOut, "Hulls found: " + to_string(contours.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::imshow("Hulls found - Cam #" + to_string(usbCameraNum), hullOut);
#endif

		// Final image
		cv::Mat final;
		curRawFrame.copyTo(final);

		// Draw crosshairs
		cv::line(final, cv::Point(final.cols / 2, 0), cv::Point(final.cols / 2, final.rows), (lockAcquired() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255)), 1);
		cv::line(final, cv::Point(0, final.rows / 2), cv::Point(final.cols, final.rows / 2), (lockAcquired() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255)), 1);

		std::vector<cv::Rect> rects(hulls.size());				// Bounding rectangles around contours
		std::vector<cv::RotatedRect> rotRects(hulls.size());	// Rotated bounding rectangles around contours
		if (hulls.size() > 0) {
			for (int i = 0; i < hulls.size(); i++) {
				cv::drawContours(final, hulls, i, cv::Scalar(0, 0, 255), 5);

				cv::Rect conRect = cv::boundingRect(hulls[i]);
				rects[i] = conRect;
				rotRects[i] = cv::minAreaRect(hulls[i]);

				JUtils::drawRotRect(final, rotRects[0], cv::Scalar(255, 255, 0));
				JUtils::drawRotRect(final, rotRects[1], cv::Scalar(255, 255, 0));

				cv::rectangle(final, conRect, cv::Scalar(255, 255, 0), 2);

				cv::Point2f aimPoint = aimCoordsFromPoint(centerPoint(conRect), final.size());
				double area = conRect.area();

				cv::putText(final, "Aim: "+ to_string(aimPoint.x) + ", " + to_string(aimPoint.y), centerPoint(conRect), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

				cv::putText(final, "Area: " + to_string(conRect.area()) + " = " + to_string(conRect.width) + "*" + to_string(conRect.height), centerPoint(conRect) + cv::Point2f(0, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);
			}
		}

		cv::putText(final, "Targets found: " + to_string(contours.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		if (rects.size() >= 2) {
			std::sort(rects.begin(), rects.end(), rectSortDesc); // Sort rectangles in descending order based on their area.

			// Pick out two largest targets - these will most likely be what we're looking for (Note to reader: I hate assumptions)
			cv::Rect targ1 = rects[0];
			cv::Rect targ2 = rects[1];

			// TODO: Calculate rectangle to encapsulate entire target...

			cv::Point2f cPoint1 = centerPoint(targ1);	// Calculate center point of both targets...
			cv::Point2f cPoint2 = centerPoint(targ2);

			cv::Point2f midPoint((cPoint1.x + cPoint2.x) / 2, (cPoint1.y + cPoint2.y) / 2); // Calculate mid points between centers of rects
			cv::Point2f tl(midPoint.x - 2, midPoint.y - 2);	// Calculate top-left point of center dot
			cv::Point2f tr(midPoint.x + 2, midPoint.y + 2); // Calculate bottom right point of center dot

			double t1Width = targ1.width;
			double t1Height = targ1.height;

			double t2Width = targ2.width;
			double t2Height = targ2.height;

			if (t1Width > t1Height && t2Width > t2Height) {	// Determine vision target type (Boiler or Gear Manipulator)
				currentLock = LOCK_TYPE::BOILER;
			} else if (t1Width < t1Height && t2Width < t2Height) {
				currentLock = LOCK_TYPE::GEAR;
			}

			cv::line(final, cPoint1, cPoint2, cv::Scalar(0, 0, 255), 1); // Draw a line between center point of vision targets
			cv::rectangle(final, tl, tr, cv::Scalar(255, 0, 0), 3); // Draw a point at center of line drawn above

			cv::Point2f midPointNormal = aimCoordsFromPoint(midPoint, final.size()); // Calculate the normalized mid point
			cv::putText(final, "Mid: "+ to_string(midPointNormal.x) + ", " + to_string(midPointNormal.y), midPoint, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

			solution = midPointNormal.x; // Calculate a solution for turning the robot. This value will ultimately be plugged into a PID loop with inputs of -1 to 1 controlling the rotation rate of the robot.
		} else {
			currentLock = LOCK_TYPE::NO_LOCK;
		}

		cv::putText(final, "Solution: " + to_string(solution), cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cFrameRate = cv::getTickFrequency() / (cv::getTickCount() - start);
		cv::putText(final, "FPS: " + to_string(cFrameRate), cvPoint(3, 45), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::putText(final, "Target Locked: " + lockTypeToString(currentLock), cvPoint(3, 60), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		string status = "Disconnected";
		if (nt::GetConnections().size() > 0) status = "Connected";
		cv::putText(final, "roboRio: " + status + " | SMblyRequired 2017 - Authored by Josh Ferrara '15", cvPoint(3, final.rows - 8), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		if (!headless) cv::imshow("CV Monitor - Cam #" + to_string(usbCameraNum), final);
		final.copyTo(algOutput);

		return solution;
    }

    cv::Mat& getVisionOutput() {
    	return algOutput;
    }

    double getSolution() {          // Returns current calculated solution
        return solution;
    }

    double getFrameRate() {         // Returns current frame rate
        return cFrameRate;          
    }

    LOCK_TYPE getLock() {			// Returns current lock type
    	return currentLock;
    }

    bool lockAcquired() {           // Returns true if target lock is acquired
        return (currentLock != LOCK_TYPE::NO_LOCK);
    }
private:
    struct {
    	bool operator() (cv::Rect a, cv::Rect b) {
    		return b.area() < a.area();
    	}
    } rectSortDesc;					// Custom sort object to sort cv::Rects in descending order

    void finishInit() {				// Anything that needs to be done before initialization is finalized
    	initialized = true;
    }

    void readFrame() {              // Reads a single frame from a video source
        if (curCapMethod == CAP_TYPE::USB) {
            if(!usbCamera.read(curRawFrame)) {
                throw "Error reading frame from camera.";
            }
        }
    }

    double aimCoords(double pos, double res) {
    	return (pos - (res / 2)) / (res / 2);
    }

    cv::Point2f aimCoordsFromPoint(cv::Point2f point, cv::Size res) {
    	return cv::Point2f(aimCoords(point.x, (double)res.width), aimCoords(point.y, (double)res.height));
    }

    cv::Point2f centerPoint(cv::Rect rect) {
    	return cv::Point2f(rect.x + (rect.width / 2), rect.y + (rect.height / 2));
    }

    double fcMinArea = 80;
    void filterContours(std::vector<std::vector<cv::Point>> &inputContours, std::vector<std::vector<cv::Point>> &outputContours) {
    	std::vector<cv::Point> hull;
    	outputContours.clear();
    	for (std::vector<cv::Point> contour : inputContours) {
    		cv::Rect bb = cv::boundingRect(contour);
    		double area = cv::contourArea(contour);
    		if (area < fcMinArea) continue;

    		outputContours.push_back(contour);
    	}
    }

    cv::Mat curRawFrame;            // Raw frame from the video source
    cv::Mat algOutput;				// Final frame for display in UI

    enum CAP_TYPE {                 // Capture type
        USB,
        NETWORK
    };

    bool initialized = false;       // Are we ready to process images?
    bool headless = false;			// Headless mode will disable imgshows

    double hslHue[2] = {60, 75};    // Hue range
    double hslSat[2] = {240, 255};  // Saturation range
    double hslLum[2] = {80, 150};   // Luminescence range

    double cFrameRate = 0.0;        // Current framerate/processing speed

    int usbCameraNum = -1;          // USB camera number
    cv::VideoCapture usbCamera;     // USB camera object

    string networkAddress;          // Networked camera address

    JVision::CAP_TYPE curCapMethod = CAP_TYPE::USB;

    double solution = 0.0;                // Current solution.
    JVision::LOCK_TYPE currentLock = LOCK_TYPE::NO_LOCK;
};
