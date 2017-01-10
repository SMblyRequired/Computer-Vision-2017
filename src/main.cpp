/*
 * main.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: josh ferrara
 */

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <exception>
#include <string>

#include "SMblyTwoKSeventeen.h"

using namespace std;

double aimCoords(double pos, double res) {
	return (pos - (res / 2)) / (res / 2);
}

cv::Point2f aimCoordsFromPoint(cv::Point2f point, cv::Size res) {
	return cv::Point2f(aimCoords(point.x, (double)res.width), aimCoords(point.y, (double)res.height));
}

cv::Point2f centerPoint(cv::Rect rect) {
	return cv::Point2f(rect.x + (rect.width / 2), rect.y + (rect.height / 2));
}

int main(int argc, char **argv) {
	cout << "Starting OpenCV algo 2k17..." << endl;

	// Config vars
	bool visualSteps = false; // Will show image window for each processing step

	double blurRadius = 13.51; // Blue radius for blur step

	double hslHue[] = {40.1, 80.3}; // Hue range
	double hslSat[] = {200.0, 255.0}; // Saturation range
	double hslLum[] = {40.6, 255.0}; // Luminescence range

	bool externalContoursOnly = true;
	// End config vars

	bool abort = false;
	double paused = false;

	cv::VideoCapture webcam;
	if (!webcam.open(0)) {
		cout << "Error - cannot open webcam!" << endl;
		exit(0);
	}

	cv::Mat curFrame;
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

		if (visualSteps) cv::imshow("Webcam Unprocessed", curFrame);

		// Blur image
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

		// HSL threshold for vision target filtering
		cv::Mat hslOut;
		cv::cvtColor(blurred, hslOut, cv::COLOR_BGR2HLS); // Convert from BGS to HLS
		cv::inRange(hslOut, cv::Scalar(hslHue[0], hslLum[0], hslSat[0]), cv::Scalar(hslHue[1], hslLum[1], hslSat[1]), hslOut); // Copy all points within range
		if (visualSteps) {
			cv::Mat hslCpy;
			hslOut.copyTo(hslCpy);

			cv::putText(hslCpy, "HSL LBound {" + to_string(hslHue[0]) + ", " + to_string(hslSat[0]) + ", " + to_string(hslLum[0]) + "}", cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN,
					0.8, cv::Scalar(255, 255, 255), 1);
			cv::putText(hslCpy, "HSL UBound {" + to_string(hslHue[1]) + ", " + to_string(hslSat[1]) + ", " + to_string(hslLum[1]) + "}", cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN,
								0.8, cv::Scalar(255, 255, 255), 1);
			cv::imshow("HSL filter", hslCpy);
		}

		// Find contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(hslOut, contours, hierarchy, (externalContoursOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST), cv::CHAIN_APPROX_SIMPLE);
		if (visualSteps) {
			// cv::Mat conOut = cv::Mat::zeros(hslOut.rows, hslOut.cols, CV_64F);
			cv::Mat conOut;
			curFrame.copyTo(conOut);

			if (contours.size() > 0) {
				int i = 0;
				for (; i >= 0; i = hierarchy[i][0]) {
					cv::drawContours(conOut, contours, i, cv::Scalar(0, 0, 255), 5);
				}
			}

			cv::putText(conOut, "Contours found: " + to_string(contours.size()), cvPoint(3, 15), cv::FONT_HERSHEY_PLAIN,
											0.8, cv::Scalar(255, 255, 255), 1);

			cv::imshow("Contours found", conOut);
		}

		// Convex hulls
		std::vector<std::vector<cv::Point>> hulls(contours.size());
		for (size_t i = 0; i < contours.size(); i++) {
			cv::convexHull(cv::Mat((contours)[i]), hulls[i], false);
		}

		if (visualSteps) {
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
		}

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
			} else {
				// TODO: Calculate point between two targets
				cv::Rect targ1 = rects[0];
				cv::Rect targ2 = rects[1];

				cv::Point2f cPoint1 = centerPoint(targ1);
				cv::Point2f cPoint2 = centerPoint(targ2);

				cv::Point2f midPoint((cPoint1.x + cPoint2.x) / 2, (cPoint1.y + cPoint2.y) / 2);
				cv::Point2f tl(midPoint.x - 2, midPoint.y - 2);
				cv::Point2f tr(midPoint.x + 2, midPoint.y + 2);

				cv::line(final, cPoint1, cPoint2, cv::Scalar(0, 0, 255), 1);
				cv::rectangle(final, tl, tr, cv::Scalar(255, 0, 0), 3);

				// If midpoint is on right half, move left
				// If midpoint is on left half, move right

				cv::Point2f midPointNormal = aimCoordsFromPoint(midPoint, final.size());
				cv::putText(final, "Mid: "+ to_string(midPointNormal.x) + ", " + to_string(midPointNormal.y), midPoint, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

				solution = (midPointNormal.x < 0 ? "Strafe left" : "Strafe right");
			}
		}

		cv::putText(final, "Solution: " + solution, cvPoint(3, 30), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1);

		cv::imshow("CV Monitor", final);
	}
}
