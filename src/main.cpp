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
#include <exception>
#include <string>
#include <assert.h>

#include <networktables/NetworkTable.h>
#include <cscore.h>

#include "cxxopts.hpp"

#include "jvision.cpp"

using namespace std;

const double HORIZ_FOV = 50.6496;
double targetAz(double xPosNorm) {
	return (HORIZ_FOV / 2) * xPosNorm;
}

// VARIABLES/METHODS USED IN CALIBRATION WINDOW
int _calCurFrame = 0;
int _calLowHue   = 0;
int _calLowLum   = 0;
int _calLowSat   = 0;
int _calHighHue  = 0;
int _calHighLum  = 0;
int _calHighSat  = 0;
void _onTrackbarChangeDoNothing( int, void* ) {}
// END OF CALIBRATION VARIABLES/METHODS

int main(int argc, char **argv) {
	/*
		Notes for testing against FRC provided vision dataset for Steamworks
		*) Boiler requires less erosion
		*) HSL bounds: H[82, 93], S[158, 255], L[41, 255]
	*/

	// Command line option handling
	cxxopts::Options options("EagleVision 2017", "Computer vision algorithm for FRC Steamworks. FRC5805.");
	options.add_options()
	("C,calibration", "Enter calibration mode")
	("H,headless", "Run in headless mode")
	("h,help", "Display this help screen")
	("n,camId", "Camera ID", cxxopts::value<int>())
	("v,videoFile", "Video file for calibration", cxxopts::value<std::string>());
	options.parse(argc, argv);

	if (options.count("help") > 0) {
		cout << options.help() << endl;
		return 0;
	}

	cout << "Starting OpenCV algo 2k17 v1..." << endl;
	cout << "\tOpenCV Version: " << CV_VERSION << endl;

	bool headless = options.count("headless") > 0;
	cout << "\tRunning headless: " << (headless == 1 ? "yes" : "no") << endl;

	// Networking setup
	NetworkTable::SetTeam(5805);
	NetworkTable::SetClientMode();
	NetworkTable::Initialize();
	shared_ptr<NetworkTable> vTable = NetworkTable::GetTable("SmartDashboard");

	bool CALIBRATION = options.count("calibration") > 0;

	bool videoFile = options.count("videoFile") > 0;
	if (!videoFile && options.count("camId") == 0) {
		cout << "No camera ID provided!" << endl;
		return 0;
	}

	cv::VideoCapture fileSource;
	int numFrames = 0;

	int cameraId = 0;

	JVision cvAlgo;
	if (videoFile) {
		cvAlgo = JVision();
		std::string videoFile = options["videoFile"].as<std::string>();
		fileSource = cv::VideoCapture(videoFile);
		numFrames = fileSource.get(CV_CAP_PROP_FRAME_COUNT);
		cout << "Successfully loaded " << videoFile << " with " << numFrames << " frames." << endl;
	} else {
		cameraId = options["camId"].as<int>();
		cvAlgo = JVision(cameraId);
	}

	cvAlgo.setHeadless(headless);

	cs::CvSource cvSource = cs::CvSource("src", cs::VideoMode::PixelFormat::kMJPEG, 640, 480, 15);
	cs::MjpegServer cvMjpgServer = cs::MjpegServer("server", 1180 + cameraId);
	cvMjpgServer.SetSource(cvSource);

	cout << "MJpeg stream available at port " << (1180 + cameraId) << endl;

	if (CALIBRATION) {
		cv::namedWindow("Calibration", 0);
		
		if (videoFile) {
			cv::createTrackbar("Frame", "Calibration", &_calCurFrame, numFrames - 1, _onTrackbarChangeDoNothing);
		}
		
		cv::createTrackbar("Hue Low", "Calibration", &_calLowHue, 255, _onTrackbarChangeDoNothing);
		cv::createTrackbar("Hue High", "Calibration", &_calHighHue, 255, _onTrackbarChangeDoNothing);
		cv::createTrackbar("Sat Low", "Calibration", &_calLowSat, 255, _onTrackbarChangeDoNothing);
		cv::createTrackbar("Sat High", "Calibration", &_calHighSat, 255, _onTrackbarChangeDoNothing);
		cv::createTrackbar("Lum Low", "Calibration", &_calLowLum, 255, _onTrackbarChangeDoNothing);
		cv::createTrackbar("Lum High", "Calibration",  &_calHighLum, 255, _onTrackbarChangeDoNothing);
	}

	bool abort = false;
	bool paused = false;
	while (!abort) {
		int keyPressed = cv::waitKey(10) & 255; // ASCII Code for pressed key

		switch (keyPressed) {
		case 27: // ESC key, quit processing
			abort = true;
			break;
		case 'p': // p/P key, pause processing
		case 'P':
			paused = !paused;
			break;
		}

		if (CALIBRATION) {
			if (videoFile) {
				fileSource.set(CV_CAP_PROP_POS_FRAMES, _calCurFrame);
			}

			cvAlgo.setHueBoundLower(_calLowHue);
			cvAlgo.setHueBoundUpper(_calHighHue);
			cvAlgo.setSatBoundLower(_calLowSat);
			cvAlgo.setSatBoundUpper(_calHighSat);
			cvAlgo.setLumBoundLower(_calLowLum);
			cvAlgo.setLumBoundUpper(_calHighLum);
		}

		if (paused) continue;

		if (videoFile) {
			fileSource.set(CV_CAP_PROP_POS_FRAMES, _calCurFrame);
			cv::Mat newFrame;
			fileSource >> newFrame;
			cvAlgo.setFrame(newFrame);
		}

		double solution = cvAlgo.run();

		if (cvAlgo.lockAcquired()) {
			// cout << "Target angle: " << targetAz(solution) << endl;
		}

		cvSource.PutFrame(cvAlgo.getVisionOutput());

		vTable->PutNumber("Solution" + to_string(cameraId), solution);
		vTable->PutBoolean("Locked" + to_string(cameraId), cvAlgo.lockAcquired());
		vTable->PutString("CurTarget" + to_string(cameraId), cvAlgo.lockTypeToString(cvAlgo.getLock()));
	}
}
