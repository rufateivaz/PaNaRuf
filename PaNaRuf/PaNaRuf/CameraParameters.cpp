#include "CameraParameters.h"

/*
	This class represents the camera intrinsic and extrinsic parameters.
	* R is rotation matrix
	* K is calibration matrix
	* fx is focal length and (0,0)-th element of K matrix
	* fy is focal length and (1,1)-th element of K matrix
	* cx is horizontal principal point and it is (0,2)-th element of K matrix
	* cy is vertical principal point and (1,2)-th element of K matrix
*/
CameraParameters :: CameraParameters() {}

CameraParameters::CameraParameters(Mat K, Mat R) {
	this->K = K;
	this->R = R;
}

void CameraParameters::setR(Mat R) {	
	this->R = R.clone();
}

void CameraParameters::setK(Mat K) {
	this->K = K.clone();
}

void CameraParameters::setCx(double cx) {
	this->K.at<double>(0, 2) = cx;
}

void CameraParameters::setCy(double cy) {
	this->K.at<double>(1, 2) = cy;
}

void CameraParameters::setFx(double fx) {
	this->K.at<double>(0, 0) = fx;
}

void CameraParameters::setFy(double fy) {
	this->K.at<double>(1, 1) = fy;
}

Mat CameraParameters::getR() {
	return this->R;
}

Mat CameraParameters::getK() {
	return this->K;
}

double CameraParameters::getCx() {
	return this->K.at<double>(0, 2);
}

double CameraParameters::getCy() {
	return this->K.at<double>(1, 2);
}