#include "PairwiseMatches.h"

PairwiseMatches :: PairwiseMatches(int obj, int scene, vector<Point2d> pointsObj, vector<Point2d> pointsScene, int numberOfGoodMatches) {
	this->obj = obj;
	this->scene = scene;
	this->pointsObj = pointsObj;
	this->pointsScene = pointsScene;
	this->NumberOfInliers = 0;
	this->isHFound = false;
	this->numberOfGoodMatches = numberOfGoodMatches;
}

void PairwiseMatches::setpointsObj(vector<Point2d> pointsObj) {
	this->pointsObj = pointsObj;
}

void PairwiseMatches::setpointsScene(vector<Point2d> pointsScene) {
	this->pointsObj = pointsScene;
}

vector<Point2d> PairwiseMatches::getpointsObj() {
	return this->pointsObj;
}

vector<Point2d> PairwiseMatches::getpointsScene() {
	return this->pointsScene;
}

void PairwiseMatches::computeH() {
	/*
		Estimates homography matrix based on RANSAC.
	*/
	CustomHomographyEstimator homographyEstimator = CustomHomographyEstimator();
	homographyEstimator.EstimateHomography(pointsObj, pointsScene, H, isHFound, NumberOfInliers);
}

bool PairwiseMatches::niceHomography()
{
	/*
		Checks if the homography matrix is good or not. (Based on book "Multiple View Geometry")
		* False (Orientation-Nonpreserving) : If determinant of top-left 2x2 matrix in H is less than 0.
		* False (Orientation-Reversing) : If determinant of H is less than 0.
		* True : Otherwise.
	*/

	double det = H.at<double>(0, 0) * H.at<double>(1, 1) - H.at<double>(1, 0) * H.at<double>(0, 1);
	if (det < 0)
		return false;

	if (determinant(H) < 0)
		return false;
	return true;
	
}

Mat PairwiseMatches::getH() {
	return this->H;
}

void PairwiseMatches::setH(Mat H) {
	this->H = H;
}

int PairwiseMatches::getObj() {
	return this->obj;
}

void PairwiseMatches::setObj(int obj) {
	this->obj = obj;
}

void PairwiseMatches::setScene(int scene) {
	this->scene = scene;
}

int PairwiseMatches::getScene() {
	return this->scene;
}

bool PairwiseMatches::isHomographyFound() {
	return this->isHFound;
}

int PairwiseMatches::getNumberOfInliers() {
	return this->NumberOfInliers;
}

int PairwiseMatches::getNumberOfGoodMatches() {
	return this->numberOfGoodMatches;
}