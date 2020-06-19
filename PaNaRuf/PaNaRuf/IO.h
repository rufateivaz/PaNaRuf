#ifndef  IO_H  
#define  IO_H 

#include <iostream>
#include <opencv2/core.hpp>
#include <fstream>
#include "PairwiseMatches.h"
#include "PanoramaType.h"
#include "CameraParameters.h"
using namespace std;
using namespace cv;

/*
	The class below handles the important Input/Output issues.
*/
class IO {

public:
	/*
		Reads inputs from argv and stores in "image_names"
	*/
	void readImageNames(vector<string>& image_names, char* argv[]);
	
	/*
		The below functions are for printing error/results/information to the user.
	*/
	void writeFieldOfViewError();

	void writeTypeOfPanoError();

	void provideCorrectInputError();

	void readingError(string image_name);

	void NoEnoughImagesException();

	void NoEnoughPairedImagesException();

	void StartPairwiseMatches();

	void StartGettingRectilinearImages();

	void StartFindingRelations();

	void StartPairwiseHomography();

	void StartConnectingImagePairs();

	void ShowConnectedImagePairs(vector<PairwiseMatches> pairs, vector<string> image_names);

	void StartFindingTransformationMatrix();

	void printTransformationMatrix(vector<Mat> Ms);

	void StartFindingCameraParameters();

	void printCameraParameters(vector<CameraParameters> cameraParams);
	
	void StartApplyingCylindricalWarping();

	void StartApplyingPerspectiveWarping();

	void StartApplyingSphericalWarping();

	void StartApplyingBlending();

	void prepareOutputImage(PanoramaType panoType, Mat result);
};
#endif