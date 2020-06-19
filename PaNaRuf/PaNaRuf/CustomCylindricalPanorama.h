#ifndef  CUSTOM_CYLINDRICAL_PANORAMA_H  
#define  CUSTOM_CYLINDRICAL_PANORAMA_H 

#include<iostream>
#include <omp.h>
#include "CameraParameters.h"
#include "IO.h"
#include "ComputeFeatures.h"
#include "CustomCameraParameterEstimation.h"
#include "Blending.h"
#include "Utils.h"
#include "CustomRelationFinder.h"

using namespace std;
using namespace cv; 
using namespace detail;
using namespace samples;

/*
This program :
	* Gets images as input
	* Finds each of the image's features
	* Estimates homography among them if there are mathcing points among them.
	* Estimates Camera calibration & rotation matrices from homography.
	* Applies cylindrical warping to each image.
	* Using the multi-band blending algorithm, stitches the warped images. 
*/
class CustomCylindricalPanorama {
public:

	// Utils object has some common helper functions. Therefore, we declare this object in all warping algorithms, to use them.
	Utils utils;

	// Gets input and prints necessary outputs during and at the end of the execution.
	IO input_output;

	// Finds all of the matching relations among the images.
	CustomRelationFinder relationFinder;

	// Applies multi-band blending algorithm to smoothly stitch the images.
	Blending blending;

	/*
		Start of the algorithm here ...
	*/
	int applyCustomCylindricalWarping(vector<String> image_names);
	
	/*
		Step by step warping operations are operated in this function.
	*/
	void Warping(vector<Mat> images, vector<CameraParameters> cameraParams, vector<Mat>& warped_images, vector<Mat>& warped_masks, vector<Point>& corners, vector<Size>& sizes);

	/*
		In this function , we apply forward warping operations to source image
		and we get the size of the destination (cylindrical) image.
	*/
	Rect forwardWarping(Mat image, Mat K, Mat R);	

	/*
		The function below applies backward mapping to each cylindrical image points,
		finds the corresponding pixel value in the source image, and gives this pixel value
		to the current pixel in cylindrical image using bilinear interpolation method.
	*/
	void BackwardWarping(Mat image, Rect roi, Mat& warped_image, Mat& warped_mask, Mat K, Mat R);
};

#endif 