/*
 * main.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: josh ferrara
 */

#define VISUALSTEPS

#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <exception>
#include <string>
#include <assert.h>

using namespace std;

enum LOCK_TYPE {
	NO_LOCK,
	GEAR,
	BOILER
};

string lockTypeToString(LOCK_TYPE cur) {
	if (cur == NO_LOCK) return "UNLOCKED";
	return (cur == BOILER ? "BOILER" : "GEAR");
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

// Don't declare global variables -Josh
int mouseX = -1;
int mouseY = -1;
static void onMouse(int event, int x, int y, int, void*) {
	if (event == 0) { // cv::MouseEventTypes::EVENT_MOUSEMOVE
		mouseX = x;
		mouseY = y;
	}
}

// Don't declare global variables -Josh
double minArea = 10;
void filterContours(std::vector<std::vector<cv::Point>> &inputContours, std::vector<std::vector<cv::Point>> &outputContours) {
	std::vector<cv::Point> hull;
	outputContours.clear();
	for (std::vector<cv::Point> contour : inputContours) {
		cv::Rect bb = cv::boundingRect(contour);
		double area = cv::contourArea(contour);
		if (area < minArea) continue;

		outputContours.push_back(contour);
	}
}

int main(int argc, char **argv) {
	cout << "Starting OpenCV algo 2k17..." << endl;

	cout << "CUDA enabled GPU's found: " << cv::gpu::getCudaEnabledDeviceCount() << endl;
	assert(cv::gpu::getCudaEnabledDeviceCount() > 0);

	// Config vars
	bool visualSteps = true; // Will show image window for each processing step

	double blurRadius = 13.51; // Blue radius for blur step

	double hslHue[] = {60, 75}; // Hue range
	double hslSat[] = {240, 255}; // Saturation range
	double hslLum[] = {80, 150}; // Luminescence range

	bool externalContoursOnly = true;
	// End config vars

	// Random variables
	LOCK_TYPE curLock = LOCK_TYPE::NO_LOCK;
	// End random variables

#ifdef VIEWSTEPS
	cv::namedWindow("Webcam Unprocessed HSL");
	cv::setMouseCallback("Webcam Unprocessed HSL", onMouse, 0);
#endif

	bool abort = false;
	double paused = false;

	cv::VideoCapture webcam;
	if (!webcam.open(0)) {
		cout << "Error - cannot open webcam!" << endl;
		exit(0);
	}

	cv::Mat curFrame;
	webcam.read(curFrame);
	cout << "Image resolution: " << curFrame.cols << "x" << curFrame.rows << endl;

	while (webcam.read(curFrame) && !abort) {
		int keyPressed = cv::waitKey(10) & 255; // ASCII Code for pressed key

		switch (keyPressed) {
		case 27: // ESC key, quit processing
			abort = true;
			break;
		case 98: // b key, lessen blur
			blurRadius -= 0.25;
			break;
		case 66: // B key, increase blur
			blurRadius += 0.25;
			break;
		case 80: // p/P key, pause processing
		case 112:
			paused = !paused;
			break;
		}

		if (paused) continue;

		int64 start = cv::getTickCount();

#ifdef VISUALSTEPS
			cv::Mat hslOut2;
			cv::cvtColor(curFrame, hslOut2, cv::COLOR_BGR2HLS); // Convert from BGS to HLS

			cv::Vec3b colorAtZeroZero = hslOut2.at<cv::Vec3b>(cv::Point(mouseX, mouseY));
			cv::putText(hslOut2, "Color at " + to_string(mouseX) + ", " + to_string(mouseY) + " (HLS): {" + to_string(colorAtZeroZero[0]) + ", " + to_string(colorAtZeroZero[1]) + ", " + to_string(colorAtZeroZero[2]) + "}",
					cvPoint(0, 45), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(80, 255, 255));

			cv::imshow("Webcam Unprocessed HSL", hslOut2);
#endif

		// Blur image
		/*
		cv::Mat blurred;
		int radius = (int)(blurRadius + 0.5);
		cv::medianBlur(curFrame, blurred, 2 * radius + 1);
		if (visualSteps) {
			cv::Mat blurCpy;
			cv::cvtColor(blurred, blurCpy, cv::COLOR_BGR2HLS); // Convert from BGS to HLS

			cv::Vec3b colorAtZeroZero = blurCpy.at<cv::Vec3b>(cv::Point(10, 10));

			cv::putText(blurCpy, "Color at zero (HLS): {" + to_string(colorAtZeroZero[0]) + ", " + to_string(colorAtZeroZero[1]) + ", " + to_string(colorAtZeroZero[2]) + "}",
					cvPoint(0, 13), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 255, 255));

			cv::imshow("Webcam blurred in HLS", blurCpy);
		}
		*/

		// HSL threshold for vision target filtering
		cv::Mat hslOut;
		cv::cvtColor(curFrame, hslOut, cv::COLOR_BGR2HLS); // Convert from BGS to HLS
		cv::inRange(hslOut, cv::Scalar(hslHue[0], hslLum[0], hslSat[0]), cv::Scalar(hslHue[1], hslLum[1], hslSat[1]), hslOut); // Copy all points within range
#ifdef VISUALSTEPS
			cv::Mat hslCpy;
			hslOut.copyTo(hslCpy);

			cv::putText(hslCpy, "HSL LBound {" + to_string(hslHue[0]) + ", " + to_string(hslSat[0]) + ", " + to_string(hslLum[0]) + "}", cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN,
					0.8, cv::Scalar(255, 255, 255), 1);
			cv::putText(hslCpy, "HSL UBound {" + to_string(hslHue[1]) + ", " + to_string(hslSat[1]) + ", " + to_string(hslLum[1]) + "}", cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN,
								0.8, cv::Scalar(255, 255, 255), 1);

			cv::imshow("HSL filter", hslCpy);
#endif

		// Find contours
		std::vector<std::vector<cv::Point>> contoursUnfiltered;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(hslOut, contoursUnfiltered, hierarchy, (externalContoursOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST), cv::CHAIN_APPROX_SIMPLE);
#ifdef VISUALSTEPS
			// cv::Mat conOut = cv::Mat::zeros(hslOut.rows, hslOut.cols, CV_64F);
			cv::Mat conOut;
			curFrame.copyTo(conOut);

			if (contoursUnfiltered.size() > 0) {
				int i = 0;
				for (; i >= 0; i = hierarchy[i][0]) {
					cv::drawContours(conOut, contoursUnfiltered, i, cv::Scalar(0, 0, 255), 5);
				}
			}

			cv::putText(conOut, "Unfiltered contours found: " + to_string(contoursUnfiltered.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN,
											0.8, cv::Scalar(255, 255, 255), 1);

			cv::imshow("Contours found", conOut);
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
			curFrame.copyTo(hullOut);

			if (hulls.size() > 0) {
				for (int i = 0; i < hulls.size(); i++) {
					cv::drawContours(hullOut, hulls, i, cv::Scalar(0, 0, 255), 5);

					cv::Rect conRect = cv::boundingRect(hulls[i]);
					cv::rectangle(hullOut, conRect, cv::Scalar(255, 255, 0), 2);
				}
			}

			cv::putText(hullOut, "Hulls found: " + to_string(contours.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN,
											0.8, cv::Scalar(255, 255, 255), 1);

			cv::imshow("Hulls found", hullOut);
#endif

		// Final image
		cv::Mat final;
		curFrame.copyTo(final);

		cv::line(final, cv::Point(final.cols / 2, 0), cv::Point(final.cols / 2, final.rows), cv::Scalar(0, 255, 0), 1);
		cv::line(final, cv::Point(0, final.rows / 2), cv::Point(final.cols, final.rows / 2), cv::Scalar(0, 255, 0), 1);

		std::vector<cv::Rect> rects(hulls.size());
		if (hulls.size() > 0) {
			for (int i = 0; i < hulls.size(); i++) {
				cv::drawContours(final, hulls, i, cv::Scalar(0, 0, 255), 5);

				cv::Rect conRect = cv::boundingRect(hulls[i]);
				rects[i] = conRect;

				cv::rectangle(final, conRect, cv::Scalar(255, 255, 0), 2);

				cv::Point2f aimPoint = aimCoordsFromPoint(centerPoint(conRect), final.size());
				double area = conRect.area();

				cv::putText(final, "Aim: "+ to_string(aimPoint.x) + ", " + to_string(aimPoint.y), centerPoint(conRect), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);
			}
		}

		cv::putText(final, "Targets found: " + to_string(contours.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		// TODO: We need to have a filter for our contours and hulls...

		string solution = "N/A";

		// We need to sort through the targets and find the one on the left, and the one on the right...
		if (rects.size() > 0) {
			if (rects.size() == 1) {
				// We only have one target, meaning one is outside our FOV. Determine move direction.
				cv::Rect cTarg = rects[0];
			} else if (rects.size() == 2) { // Targets acquired (maybe...)
				// TODO: Calculate point between two targets
				cv::Rect targ1 = rects[0];
				cv::Rect targ2 = rects[1];

				cv::Point2f cPoint1 = centerPoint(targ1);
				cv::Point2f cPoint2 = centerPoint(targ2);

				cv::Point2f midPoint((cPoint1.x + cPoint2.x) / 2, (cPoint1.y + cPoint2.y) / 2);
				cv::Point2f tl(midPoint.x - 2, midPoint.y - 2);
				cv::Point2f tr(midPoint.x + 2, midPoint.y + 2);

				double t1Width = targ1.width;
				double t1Height = targ1.height;

				double t2Width = targ2.width;
				double t2Height = targ2.height;

				if (t1Width > t1Height && t2Width > t2Height) {
					curLock = LOCK_TYPE::BOILER;
				} else if (t1Width < t1Height && t2Width < t2Height) {
					curLock = LOCK_TYPE::GEAR;
				}

				cv::line(final, cPoint1, cPoint2, cv::Scalar(0, 0, 255), 1);
				cv::rectangle(final, tl, tr, cv::Scalar(255, 0, 0), 3);

				// If midpoint is on right half, move left
				// If midpoint is on left half, move right

				cv::Point2f midPointNormal = aimCoordsFromPoint(midPoint, final.size());
				cv::putText(final, "Mid: "+ to_string(midPointNormal.x) + ", " + to_string(midPointNormal.y), midPoint, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

				solution = to_string(midPointNormal.x);
			}
		} else {
			curLock = LOCK_TYPE::NO_LOCK;
		}

		cv::putText(final, "Solution: " + solution, cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
		cv::putText(final, "FPS: " + to_string(fps), cvPoint(3, 45), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::putText(final, "Target Locked: " + lockTypeToString(curLock), cvPoint(3, 60), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::imshow("CV Monitor", final);
	}
}
