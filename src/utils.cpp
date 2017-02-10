/*
 * utils.cpp
 *
 *  Created on: Feb 10, 2017
 *      Author: josh ferrara
 */

class JUtils {
public:
	double aimCoords(double pos, double res) {
		return (pos - (res / 2)) / (res / 2);
	}
}