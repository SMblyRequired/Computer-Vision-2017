#include "SMblyTwoKSeventeen.h"
/**
* Initializes a SMblyTwoKSeventeen.
*/

namespace grip {

SMblyTwoKSeventeen::SMblyTwoKSeventeen() {
}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method. 
*
*/
void SMblyTwoKSeventeen::process(cv::Mat source0){
	//Step Resize_Image0:
	//input
	cv::Mat resizeImageInput = source0;
	double resizeImageWidth = 640.0;  // default Double
	double resizeImageHeight = 480.0;  // default Double
	int resizeImageInterpolation = cv::INTER_CUBIC;
	resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, this->resizeImageOutput);
	//Step Blur0:
	//input
	cv::Mat blurInput = resizeImageOutput;
	BlurType blurType = grip::MEDIAN;
	double blurRadius = 1;  // default Double
	blur(blurInput, blurType, blurRadius, this->blurOutput);

	cv::imshow("Blurred", this->blurOutput);

	//Step HSL_Threshold0:
	//input
	cv::Mat hslThresholdInput = blurOutput;
	double hslThresholdHue[] = {56.09570528573521, 117.30043433526004};
	double hslThresholdSaturation[] = {93.04794925508544, 255.0};
	double hslThresholdLuminance[] = {59.62230215827338, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);


	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hslThresholdOutput;
	bool findContoursExternalOnly = true;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);


	//Step Convex_Hulls0:
	//input
	std::vector<std::vector<cv::Point> > convexHullsContours = findContoursOutput;
	convexHulls(convexHullsContours, this->convexHullsOutput);
	//Step Filter_Contours0:
	//input
	std::vector<std::vector<cv::Point> > filterContoursContours = convexHullsOutput;
	double filterContoursMinArea = 500.0;  // default Double
	double filterContoursMinPerimeter = 0.0;  // default Double
	double filterContoursMinWidth = 0.0;  // default Double
	double filterContoursMaxWidth = 1000.0;  // default Double
	double filterContoursMinHeight = 0.0;  // default Double
	double filterContoursMaxHeight = 1000.0;  // default Double
	double filterContoursSolidity[] = {0.0, 100};
	double filterContoursMaxVertices = 1000000.0;  // default Double
	double filterContoursMinVertices = 0.0;  // default Double
	double filterContoursMinRatio = 0.0;  // default Double
	double filterContoursMaxRatio = 1000.0;  // default Double
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);
}

/**
 * This method is a generated getter for the output of a Resize_Image.
 * @return Mat output from Resize_Image.
 */
cv::Mat* SMblyTwoKSeventeen::getresizeImageOutput(){
	return &(this->resizeImageOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* SMblyTwoKSeventeen::getblurOutput(){
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a HSL_Threshold.
 * @return Mat output from HSL_Threshold.
 */
cv::Mat* SMblyTwoKSeventeen::gethslThresholdOutput(){
	return &(this->hslThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* SMblyTwoKSeventeen::getfindContoursOutput(){
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Convex_Hulls.
 * @return ContoursReport output from Convex_Hulls.
 */
std::vector<std::vector<cv::Point> >* SMblyTwoKSeventeen::getconvexHullsOutput(){
	return &(this->convexHullsOutput);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* SMblyTwoKSeventeen::getfilterContoursOutput(){
	return &(this->filterContoursOutput);
}
	/**
	 * Scales and image to an exact size.
	 *
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	void SMblyTwoKSeventeen::resizeImage(cv::Mat &input, double width, double height, int interpolation, cv::Mat &output) {
		cv::resize(input, output, cv::Size(width, height), 0.0, 0.0, interpolation);
	}

	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void SMblyTwoKSeventeen::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
	}
	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param lum The min and max luminance.
	 * @param output The image in which to store the output.
	 */
	//void hslThreshold(Mat *input, double hue[], double sat[], double lum[], Mat *out) {
	void SMblyTwoKSeventeen::hslThreshold(cv::Mat &input, double hue[], double sat[], double lum[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HLS);
		cv::inRange(out, cv::Scalar(hue[0], lum[0], sat[0]), cv::Scalar(hue[1], lum[1], sat[1]), out);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void SMblyTwoKSeventeen::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Compute the convex hulls of contours.
	 *
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	void SMblyTwoKSeventeen::convexHulls(std::vector<std::vector<cv::Point> > &inputContours, std::vector<std::vector<cv::Point> > &outputContours) {
		std::vector<std::vector<cv::Point> > hull (inputContours.size());
		outputContours.clear();
		for (size_t i = 0; i < inputContours.size(); i++ ) {
			cv::convexHull(cv::Mat((inputContours)[i]), hull[i], false);
		}
		outputContours = hull;
	}


	/**
	 * Filters through contours.
	 * @param inputContours is the input vector of contours.
	 * @param minArea is the minimum area of a contour that will be kept.
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
	 * @param minWidth minimum width of a contour.
	 * @param maxWidth maximum width.
	 * @param minHeight minimum height.
	 * @param maxHeight  maximimum height.
	 * @param solidity the minimum and maximum solidity of a contour.
	 * @param minVertexCount minimum vertex Count of the contours.
	 * @param maxVertexCount maximum vertex Count.
	 * @param minRatio minimum ratio of width to height.
	 * @param maxRatio maximum ratio of width to height.
	 * @param output vector of filtered contours.
	 */
	void SMblyTwoKSeventeen::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
		std::vector<cv::Point> hull;
		output.clear();
		for (std::vector<cv::Point> contour: inputContours) {
			cv::Rect bb = boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			double area = cv::contourArea(contour);
			if (area < minArea) continue;
			if (arcLength(contour, true) < minPerimeter) continue;
			cv::convexHull(cv::Mat(contour, true), hull);
			double solid = 100 * area / cv::contourArea(hull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
			double ratio = bb.width / bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.push_back(contour);
		}
	}



} // end grip namespace

