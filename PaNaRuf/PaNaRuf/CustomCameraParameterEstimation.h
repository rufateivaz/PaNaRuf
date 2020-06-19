#ifndef  CUSTOM_CAMERA_PARAMETERS_ESTIMATION_H
#define  CUSTOM_CAMERA_PARAMETERS_ESTIMATION_H 

#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "CameraParameters.h"
#include "PairwiseMatches.h"

using namespace std;
using namespace cv;
using namespace detail;
using namespace xfeatures2d;
using namespace samples;

class CustomCameraParameterEstimation {

public:
	/*
`		Based on Szeliski's book (page 57), we estimate focals using
		homographies and orthonormality property of rotation matrix.
		We find f0 and f1, then we add sqrt(f0*f1) to focals list.
		If we can not find any of them, we ignore the current case.
	*/
	void EstimateFocals(vector<PairwiseMatches> pairs, vector<double>& focals);

	/*
		Based on Szeliski's book (page 57), we sort the estimated focals and
		we choose the median as the final focal length of all cameras.
		We assume that all images have been taken by the same camera.
	*/
	double getFocalByMedian(vector<double> focals);


	/*
		If we can not estimate the focals via the method (from Szeliski's book), then 
		by default we take the focal length as (width + height) of the image.
	*/
	double getFocalByDefault(vector<Mat> images);


	/*
		In this function, we set all of the corresponding camera parameters:
		* width/2, height/2 are principal points at K(0,2) and K(1,2)
		* f is focal length at K(0,0) and K(1,1) 
	*/
	void setCalibrationMatrices(vector<Mat> images, vector<CameraParameters>& cameras, double focal);
	/*
		* Below, we choose middle image, and align the other images around it.
			So, first of all, we choose Identity matrix (no move) as its rotation matrix.
		* Then, based on pairwise homography, estimated calibration matrix, we
			estimate the rotation matrices by :
			H = K1 * R1 * R0 ^-1 * K0^-1 : where 0 is object, 1 is scene.
	*/
	void setRotationMatrices(vector<Mat> images, vector<PairwiseMatches> pairs, vector<CameraParameters>& cameras);



};
#endif
