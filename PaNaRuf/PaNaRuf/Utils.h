#ifndef  UTILS_H  
#define  UTILS_H 

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D

using namespace std;
using namespace cv;

class Utils {

public:

	/*
		We add all of the images to "images" vector by reading image names from the vector "image_names".
		Additionally, we decrease the size of the image in order to improve the overall complexity.
		
		* If all read operations are done without any trouble, then the function returns -1.
		* Else it returns the index of the image making problem. (we need this information to
		  print out for the user to see which image is problematic.
	*/
	int addInputImages(vector<String> image_names, vector<Mat>& images);

	/*	This function creates self-reflected images around the current image.
		We are doing this to get rid of black colors around the warped images
		which is important for applying multi-band blending to all warped images in the end.
	*/
	void addReflectiveImagesAround(Mat image_sri, Mat& original_image);

	/*
		The function below takes (x,y) image coordinates and calculates the pixel value
		using bilinear interpolation method.
	*/
	Vec3b BilinearInterpolation(double x, double y, Mat img);
};
#endif
#endif 