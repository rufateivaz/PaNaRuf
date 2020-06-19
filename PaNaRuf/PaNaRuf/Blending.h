#ifndef  BLENDING_H  
#define  BLENDING_H 

#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"

using namespace std;
using namespace cv;
using namespace detail;
using namespace xfeatures2d;
using namespace samples;

class Blending {

public:

	/*
		This function finds seams among the warped images.
		Then, it stitches those images using multi-band blending algorithm.

		Note: Thanks to OpenCV, we used its open-source  methods and used them in the function below
			  and arranged for our software.
	*/
	void applyMultiBandBlending(vector<Mat> warped_masks, vector<Mat> warped_images,
		vector<Point> corners, vector<Size> sizes, Mat& result
	);
};

#endif