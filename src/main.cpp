/*
 * main.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: josh ferrara
 */

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

#include "jvision.cpp"

using namespace std;

const double HORIZ_FOV = 50.6496;
double targetAz(double xPosNorm) {
	return (HORIZ_FOV / 2) * xPosNorm;
}

int main(int argc, char **argv) {
	if (argc <= 1) {
		cout << "Usage: eaglevision [cameraId] [headless 0/1]" << endl;
		return 0;
	}

	cout << "Starting OpenCV algo 2k17 v1..." << endl;
	cout << "\tOpenCV Version: " << CV_VERSION << endl;
	// assert(CV_VERSION == "3.2.0"); // This code has only been tested to work on OpenCV 3.2.0

	bool headless = (argc == 3 && stoi(argv[2]) == 1);
	int cameraId = stoi(argv[1]);

	NetworkTable::SetTeam(5805);
	NetworkTable::SetClientMode();
	NetworkTable::Initialize();

	shared_ptr<NetworkTable> vTable = NetworkTable::GetTable("SmartDashboard");

	JVision cvAlgo(cameraId); // Initialize the algo for another camera.
	cvAlgo.setHeadless(headless);

	cs::CvSource cvSource = cs::CvSource("src", cs::VideoMode::PixelFormat::kMJPEG, 640, 480, 15);
	cs::MjpegServer cvMjpgServer = cs::MjpegServer("server", 1180 + cameraId);
	cvMjpgServer.SetSource(cvSource);

	cout << "MJpeg stream available at port " << (1180 + cameraId) << endl;

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

		if (paused) continue;
		double solution = cvAlgo.run();

		if (cvAlgo.lockAcquired()) {
			cout << "Target angle: " << targetAz(solution) << endl;
		}

		cvSource.PutFrame(cvAlgo.getVisionOutput());

		vTable->PutNumber("Solution" + to_string(cameraId), solution);
		vTable->PutBoolean("Locked" + to_string(cameraId), cvAlgo.lockAcquired());
		vTable->PutString("CurTarget" + to_string(cameraId), cvAlgo.lockTypeToString(cvAlgo.getLock()));
	}
}
