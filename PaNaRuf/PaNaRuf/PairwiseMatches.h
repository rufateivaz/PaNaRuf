#ifndef  PAIRWISE_MATCHES_H  
#define  PAIRWISE_MATCHES_H 

#include <iostream>
#include <opencv2/core.hpp>
#include "CustomHomographyEstimator.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace xfeatures2d;


/*
	This class is helpful to store all datas below at once: 
	* pointsObj, pointsScene (where pointsScene = H * pointsScene)
	* obj, scene -> the index values in the images vector of the points above.
	* H -> homography matrix between two  images.
	* NumberOfInliers -> the number of inliers obtained after estimating homography matrix.
	* isHfound -> boolean variable which indicates if the homography matrix could be estimated or not
	* numberOfGoodMatches -> the number of good matches between two images.

*/

class PairwiseMatches {

private:
	vector<Point2d> pointsObj;
	vector<Point2d> pointsScene;
	int obj;
	int scene;
	Mat H;
	int NumberOfInliers;
	bool isHFound;
	int numberOfGoodMatches;
public:
	PairwiseMatches(int obj, int scene, vector<Point2d> pointsObj, vector<Point2d> pointsScene, int numberOfGoodMatches);

	void setpointsObj(vector<Point2d> pointsObj);

	void setpointsScene(vector<Point2d> pointsScene);

	vector<Point2d> getpointsObj();

	vector<Point2d> getpointsScene();

	/*
		Estimates homography matrix based on RANSAC.
	*/
	void computeH();
	
	/*
		Checks if the homography matrix is good or not. (Based on book "Multiple View Geometry")
		* False (Orientation-Nonpreserving) : If determinant of top-left 2x2 matrix in H is less than 0.
		* False (Orientation-Reversing) : If determinant of H is less than 0.
		* True : Otherwise.
	*/
	bool niceHomography();

	Mat getH();

	void setH(Mat H);

	int getObj();

	void setObj(int obj);

	void setScene(int scene);

	int getScene();

	bool isHomographyFound();

	int getNumberOfInliers();

	int getNumberOfGoodMatches();
};
#endif 