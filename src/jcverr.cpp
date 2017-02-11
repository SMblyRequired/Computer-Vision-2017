/*
 * jcverr.cpp
 *
 *  Created on: Feb 10, 2017
 *      Author: josh
 */

#include <exception>
#include <string>

class CameraUnavailable : public std::exception {
public:
	virtual const char* what() const throw() {
		return "Could not open camera for read.";
	}
};
