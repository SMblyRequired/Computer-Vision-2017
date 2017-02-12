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
#include <opencv2/gpu/gpu.hpp>
#include <exception>
#include <string>
#include <assert.h>

#include "jvision.cpp"

using namespace std;

int main(int argc, char **argv) {
	cout << "Starting OpenCV algo 2k17..." << endl;

	cout << "CUDA enabled GPU's found: " << cv::gpu::getCudaEnabledDeviceCount() << endl;
	assert(cv::gpu::getCudaEnabledDeviceCount() > 0);

	bool abort = false;
	double paused = false;

	// NetworkTable::SetTeam(5805);
	// NetworkTable::SetClientMode();
	// NetworkTable::Initialize();
	// std::shared_ptr<NetworkTable> vTable = NetworkTable::GetTable("Vision");

	JVision cvAlgo(0);	// Create a JVision object attached to USB camera 0
	// JVision cvAlgo(1);	// Initialize the algo for another camera. Moving the code into it's own class allows for easy multithreading in the future.

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

		// NetworkTable::PutNumber("Solution", solution);
		// NetworkTable::PutBoolean("Locked", cvAlgo.lockAcquired());
		// NetworkTable::PutString("CurTarget", cvAlgo.lockTypeToString(cvAlgo.getLock()));
		// TODO: Research how to display final image on driver station :thinking_face:
	}
}
