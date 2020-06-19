#include "CustomCameraParameterEstimation.h"

void CustomCameraParameterEstimation::EstimateFocals(vector<PairwiseMatches> pairs, vector<double>& focals) {

	/*
	`		Based on Szeliski's book (page 57), we estimate focals using
			homographies and orthonormality property of rotation matrix. 
			We find f0 and f1, then we add sqrt(f0*f1) to focals list.
			If we can not find any of them, we ignore the current case.
	*/

	for (int i = 0; i < pairs.size(); i++) {

		Mat H = pairs[i].getH();
		
		double h00 = H.at<double>(0, 0); double h01 = H.at<double>(0, 1); double h02 = H.at<double>(0, 2);
		double h10 = H.at<double>(1, 0); double h11 = H.at<double>(1, 1); double h12 = H.at<double>(1, 2);
		double h20 = H.at<double>(2, 0); double h21 = H.at<double>(2, 1); double h22 = H.at<double>(2, 2);

		double f0 = 0;
		double f1 = 0;


		//estimating f0
		double x1 = h00 * h10 + h01 * h11;
		double eq1 = -h02 * h12;
		if (x1 != 0) eq1 = eq1 / x1;
		else eq1 = -1;

		double x2 = h00 * h00 + h01 * h01 - (h10 * h10 + h11 * h11);
		double eq2 = (h12 * h12 - h02 * h02);
		if (x2 != 0) eq2 = eq2 / x2;
		else eq2 = -1;

		if (eq1 > 0 && eq2 > 0) {
			if (abs(x1) > abs(x2))
				f0 = sqrt(eq1);
			else f0 = sqrt(eq2);
		}

		else if (eq1 > 0 && eq2 < 0)
			f0 = sqrt(eq1);

		else if (eq1 < 0 && eq2 > 0)
			f0 = sqrt(eq2);

		// estimating f1
		x1 = h20 * h21;
		eq1 = -(h00 * h01 + h10 * h11);
		if (x1 != 0) eq1 = eq1 / x1;
		else eq1 = -1;

		x2 = h21 * h21 - h20 * h20;
		eq2 = h00 * h00 + h10 * h10 - (h01 * h01 + h11 * h11);
		if (x2 != 0) eq2 = eq2 / x2;
		else eq2 = -1;

		if (eq1 > 0 && eq2 > 0) {
			if (abs(x1) > abs(x2))
				f1 = sqrt(eq1);
			else f1 = sqrt(eq2);
		}

		else if (eq1 > 0 && eq2 < 0)
			f1 = sqrt(eq1);

		else if (eq1 < 0 && eq2 > 0)
			f1 = sqrt(eq2);

		
		if (f1 > 0 && f0 > 0)
			focals.push_back(sqrt(f1 * f0));
	
		if (f1 > 0 && f0 == 0)
			focals.push_back(f1);

		if (f0 > 0 && f1 == 0)
			focals.push_back(f0);
		

	}

}


double CustomCameraParameterEstimation::getFocalByMedian(vector<double> focals)
{
	/*
		Based on Szeliski's book (page 57), we sort the estimate focals and
		we choose the median as the final focal length of all cameras.
		We assume that all images have been taken by the same camera.
	*/
	sort(focals.begin(), focals.end());
	if (focals.size() % 2 == 0)
		return  (focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) / 2;
	else return focals[focals.size() / 2];
}

double CustomCameraParameterEstimation::getFocalByDefault(vector<Mat> images)
{	
	/*
		If we can not estimate the focals via the method (from Szeliski's book), then
		by default we take the focal length as (width + height) of the image.
	*/
	double focal = 0;
	for (int i = 0; i < images.size(); i++) {
		focal += double(images[i].cols) + double(images[i].rows);
	}
	return focal / images.size();
}



void CustomCameraParameterEstimation::setCalibrationMatrices(vector<Mat> images, vector<CameraParameters>& cameras, double focal) {
	
	/*
		In this function, we set all of the corresponding camera parameters:
		* width/2, height/2 are principal points at K(0,2) and K(1,2)
		* f is focal length at K(0,0) and K(1,1)
	*/
	
	Mat K = Mat::eye(Size(3, 3), CV_64F);
	for (int i = 0; i < cameras.size(); i++) {
		K.at<double>(0, 0) = focal;
		K.at<double>(1, 1) = focal;
		K.at<double>(0, 2) = double(images[i].cols) / 2;
		K.at<double>(1, 2) = double(images[i].rows) / 2;
		cameras[i].setK(K);
	}
}



void CustomCameraParameterEstimation::setRotationMatrices(vector<Mat> images, vector<PairwiseMatches> pairs, vector<CameraParameters>& cameras) {
	/*
		* Below, we choose middle image, and align the other images around it.
			So, first of all, we choose Identity matrix (no move) as its rotation matrix.
		* Then, based on pairwise homography, estimated calibration matrix, we
			estimate the rotation matrices by :
			H = K1 * R1 * R0 ^-1 * K0^-1 : where 0 is object, 1 is scene.
	*/
	cameras[cameras.size() / 2].setR(Mat::eye(Size(3, 3), CV_64F));
	
	for (int i = 0; i < images.size(); i++) {
		for (int j = 0; j < pairs.size(); j++) {
			if (cameras[pairs[j].getObj()].getR().empty() && !cameras[pairs[j].getScene()].getR().empty()) {
				Mat R = cameras[pairs[j].getObj()].getK().inv() * pairs[j].getH().inv() * cameras[pairs[j].getScene()].getK() * cameras[pairs[j].getScene()].getR();
				cameras[pairs[j].getObj()].setR(R);
			}
			if (!cameras[pairs[j].getObj()].getR().empty() && cameras[pairs[j].getScene()].getR().empty()) {
				Mat R = cameras[pairs[j].getScene()].getK().inv() * pairs[j].getH() * cameras[pairs[j].getObj()].getK() * cameras[pairs[j].getObj()].getR();
				cameras[pairs[j].getScene()].setR(R);
			}
		}
	}
}
