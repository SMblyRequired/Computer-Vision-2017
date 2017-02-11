/*
 * main.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: josh ferrara
 */

// #define VISUALSTEPS

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

	JVision cvAlgo(0);

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
		cvAlgo.run();
	}
}
