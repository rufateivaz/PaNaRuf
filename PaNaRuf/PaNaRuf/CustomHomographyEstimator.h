#ifndef  CUSTOM_HOMOGRAPHY_ESTIMATOR_H  
#define  CUSTOM_HOMOGRAPHY_ESTIMATOR_H 

#include<iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class CustomHomographyEstimator {
	
public :
	/*
		Based on Hartley and Zisserman (Multiple View Geometry) the normalization is to
		compute a similarity transform T that takes "points" to a new set
		of points "p_prime" such that the centroid of "p_prime" is the coordinate
		origin and their average distance from the origin is sqrt(2)
	*/
	void Normalization(vector<Point2d>& p, Mat& T_mat);

	/*
		We take three points combinations and check if the area of the triangle is 0
		If yes, then the points lie on the same line: FALSE:-> co-linearity.
		If no, then TRUE:-> no co-linearity.
		[Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By)] / 2 ?= 0
	*/
	bool isColinear(vector<Point2d> p);


	/*
		We randomly choose 4 different points to apply DLT.
		Since they are chosen randomly, we do not know
		if there exist repeating (same) index of points in the list.
		If yes, then we can not apply DLT algorithm since
		the number of points will be < 4.

		Therefore, with this function we check if we have
		randomly chosen any repeated index of points or not.

		if yes: false
		otherwise: true
	*/
	bool chosenSamePoint(vector<int> indexes);

	/*
		This function calculates distanceThreshold t where:
			t = sqrt(5.99) * sigma and sigma is standard deviation.
	*/
	double distanceThreshold(vector<double>& forward_errors, vector<double>& backward_errors, Mat H, vector<Point2d> obj, vector<Point2d> scene);

	/*
		This function computes inliers and the number of inliers that holds the inequality:
			d < sqrt(5.99) * sigma (where sigma is standard deviation)
	*/
	void computeInliers(vector<Point2d> obj, vector<Point2d> scene, Mat H, vector<int>& inliers, int& numberOfInliers);

	/*
		This function is used to estimate homography for every N >= 4 points combinations as below:
		* Using Singular Value Decomposition, solve Ah = 0 and assign the last row of Vt where U*W*Vt = SVD(A) to "h" vector.
		* Construct H (3x3) homography matrix from found "h" vector.
	*/
	void DirectLinearTransform(vector<Point2d> points1, vector<Point2d> points2, Mat& cH);


	/*
		This function operates all process for estimating the homography if exists.(based on Elan Dubrofsky)
		The algorithm is as below:
		* Iterate N times.
		* Randomly choose 4 correspondences
		* Check if chosen points are colinear or repeated:
			* If yes, randomly choose another 4 points.
			* If no, go on.
		* Normalize the set of points, by computing similarity transform T1,T2
		* Compute Homography via DLT.
		* Denormalize for getting the homography H = T2^-1 * H_prime * T1
		* Do classification (inlier/outlier) based on concurrence of each other correspondence with H
		* Choose the iteration with maximum number of inliers.
	*/
	void EstimateHomography(vector<Point2d> obj, vector<Point2d> scene, Mat& H, bool& isHfound, int& max);
};

#endif 