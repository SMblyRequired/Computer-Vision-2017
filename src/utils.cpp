/*
 * utils.cpp
 *
 *  Created on: Feb 10, 2017
 *      Author: josh ferrara
 */

#include <opencv2/core.hpp>

class JUtils {
public:
	static double aimCoords(double pos, double res) {
		return (pos - (res / 2)) / (res / 2);
	}

	static void drawRotRect(cv::Mat &output, cv::RotatedRect rect, cv::Scalar color) {
		cv::Point2f rect_points[4];
		rect.points(rect_points);
		for( int j = 0; j < 4; j++ ) {
			cv::line( output, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
		}
	}
};
