#ifndef  CUSTOM_RELATION_FINDER_H  
#define  CUSTOM_RELATION_FINDER_H 

#include <iostream>
#include <opencv2/core.hpp>
#ifdef HAVE_OPENCV_XFEATURES2D
#include "PairwiseMatches.h"
#include "CameraParameters.h"
#include "ComputeFeatures.h"
#include "CustomCameraParameterEstimation.h"

using namespace std;
using namespace cv;
using namespace detail;
using namespace xfeatures2d;
using namespace samples;

class CustomRelationFinder {

public:
	/*
		Finds relationships among rectilinear image sets.
	*/
	void findRelationsAmongImageSets(vector<vector<Mat>>& rectImagesSet, vector<vector<CameraParameters>>& rectCamerasSet, vector<String> image_names);
	
	/*
		* Extracts features of each images.
		* Finds good matching points among the images.
		* Estimates homographies using those matching points.
		* Keeps only the images creating panoramic network.
		* Removes the other images which has no relation or weak relation.
	*/
	void findRelationsAmongImages(vector<Mat>& images, vector<PairwiseMatches>& pairs);
	/*
		Removes an image if it has weaker relation with any other image, or if it has no relation with any of the images.
	*/
	void removeUnpairedImages(vector<Mat>& images, vector<PairwiseMatches>& pairs);
	
};
#endif
#endif 