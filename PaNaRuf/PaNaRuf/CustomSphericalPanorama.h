#ifndef  CUSTOM_SPHERICAL_PANORAMA_H  
#define  CUSTOM_SPHERICAL_PANORAMA_H 

#include<iostream>
#include <opencv2/core.hpp>
#include <omp.h>
#include "opencv2/imgcodecs.hpp"
#include "CameraParameters.h"
#include "IO.h"
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
	* Gets images, field of view parameters as input 
	* Derives different set of rectilinear images and corresponding camera parameters from 
		each different fisheye images. 
	* Finds the relations among the image sets via best homography matrix.
	* Updates the rotation matrices relative to each other.
	* Applies spherical warping to each rectilinear image.
	* Using the multi-band blending algorithm, stitches the warped images.
*/
class CustomSphericalPanorama {
public:
	
	double hfov; //horizontal field of view
	double vfov; //vertical field of view

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
	int applyCustomSphericalWarping(vector<String> image_names, double hfov, double vfov);
	
	/*
		converts degree to radian
	*/
	double toRadian(double degree);
	
	/*
		converts radian to degree
	*/
	double toDegree(double radian);
	
	/*
		based on the parameters given to the function,
		the function constructs 3x3 rotation matrices Rx,Ry and Rz that are rotating the 
		virtual pinhole camera around x,y and z respectively.
		Then it returns  Rx * Ry * Rz.
	*/
	Mat Rotation(double aroundX, double aroundY, double aroundZ);
	
	/*
		Below, the function constructs the corresponding camera intrinsic and 
		extrinsic parameters for each rectilinear image.
	*/
	void findCameraParameters(vector<CameraParameters>& cameraParams, vector<Mat>& perspImages, int& numberOfImages, int wd, int hd);
	
	/*
		The function below obtains multiple rectilinear images (and corresponding camera parameters K,R) from a fisheye image.
	*/
	void fish2persp(Mat& image, vector<Mat>& images, vector<CameraParameters>& cameraParams);


	/*
		Step by step warping operations are operated in this function.
	*/
	void Warping(vector<vector<Mat>> images, vector<vector<CameraParameters>> cameraParams, vector<Mat>& warped_images, vector<Mat>& warped_masks, vector<Point>& corners, vector<Size>& sizes);
	
	/*
		In this function , we apply forward warping operations to source image
		and we get the size of the destination (spherical) image.
	*/
	Rect forwardWarping(Mat image, Mat K, Mat R);
	

	/*
		The function below applies backward mapping to each spherical image points,
		finds the corresponding pixel value in the source (rectilinear) image, and 
		gives this pixel value to the current pixel in spherical image using 
		bilinear interpolation method.
	*/
	void BackwardWarping(Mat image, Rect roi, Mat& warped_image, Mat& warped_mask, Mat K, Mat R);
};
#endif