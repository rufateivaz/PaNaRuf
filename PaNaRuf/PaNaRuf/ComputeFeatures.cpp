#include "ComputeFeatures.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;
using namespace detail;
using namespace xfeatures2d;
using namespace samples;

int ComputeFeatures::imageFeatureComputer(Mat image1, Mat image2, vector<Point2d>& obj, vector<Point2d>& scene) {

	/*
		Below function, finds the features of two images by SIFT feature detector. Using those data,
		it finds matching points. Finally, it chooses good matched points and stores them in
		"obj" and "scene" points list.
		
		Note: Thanks to OpenCV,  we apply this function to find the matched points between two images.
			  Then, we use those matched points to estimate the homography.
	*/

	//-- Step 1: Detect the keypoints using SIFT Detector, compute the descriptors
	Ptr<Feature2D> finder = SIFT::create();
	ImageFeatures feature1;
	ImageFeatures feature2;
	computeImageFeatures(finder, image1, feature1);
	feature1.img_idx = 1;
	computeImageFeatures(finder, image2, feature2);
	feature2.img_idx = 2;
	MatchesInfo matchesInfo;
	Ptr<FeaturesMatcher> matcher;
	matcher = makePtr<BestOf2NearestMatcher>(false , 0.65);
	(*matcher)(feature1, feature2, matchesInfo);
	matcher->collectGarbage();

	for (int i = 0; i < matchesInfo.matches.size(); i++)
	{
		//--Get the keypoints from the good matches
		obj.push_back(feature1.keypoints[matchesInfo.matches[i].queryIdx].pt);

		scene.push_back(feature2.keypoints[matchesInfo.matches[i].trainIdx].pt);

	}
	return (int)matchesInfo.matches.size();
}