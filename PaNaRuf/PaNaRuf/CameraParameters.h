#ifndef  CAMERA_PARAMETERS_H  
#define  CAMERA_PARAMETERS_H 

#include <iostream>
#include <opencv2/core.hpp>


using namespace std;
using namespace cv;

/*
	This class represents the camera intrinsic and extrinsic parameters.
	* R is rotation matrix
	* K is calibration matrix
	* fx is focal length and (0,0)-th element of K matrix
	* fy is focal length and (1,1)-th element of K matrix
	* cx is horizontal principal point and it is (0,2)-th element of K matrix
	* cy is vertical principal point and (1,2)-th element of K matrix
*/
class CameraParameters { 

public :
	Mat R;
	Mat K;
	
	CameraParameters();
	CameraParameters(Mat K, Mat R);
	void setR(Mat R);
	void setK(Mat K);
	void setCx(double cx);
	void setCy(double cy);
	void setFx(double fx);
	void setFy(double fy);
	Mat getR();
	Mat getK();
	double getCx();
	double getCy();
};

#endif 