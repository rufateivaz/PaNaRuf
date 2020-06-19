#ifndef  COMPUTE_FEATURES_H  
#define  COMPUTE_FEATURES_H 

#include <iostream>
#include <opencv2/core.hpp>
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

using namespace std;
using namespace cv;
using namespace detail; 
using namespace xfeatures2d; 
using namespace samples; 

class ComputeFeatures {

public:

	/*
		Below function, finds the features of two images by SIFT feature detector. Using those data,
		it finds matching points. Finally, it chooses good matched points and stores them in
		"obj" and "scene" points list.
		
		Note: Thanks to OpenCV,  we apply this function to find the matched points between two images.
			  Then, we use those matched points to estimate the homography.
	*/
	int imageFeatureComputer(Mat image1, Mat image2, vector<Point2d>& obj, vector<Point2d>& scene);
};
#endif
#endif 