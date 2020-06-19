#include "Utils.h"

using namespace std;
using namespace cv;
using namespace detail;

int Utils::addInputImages(vector<String> image_names, vector<Mat>& images) {
	double work_scale = 1;
	bool is_work_scale_set = false;
	for (int i = 0; i < image_names.size(); i++) {
		Mat image = imread(image_names[i]);
		if (image.empty()) {
			return i;
		}

		if (!is_work_scale_set)
		{
			work_scale = min(1.0, sqrt(0.6 * 1e6 / image.size().area()));
			is_work_scale_set = true;
		}

		resize(image, images[i], Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
		image.release();

	}
	return -1;
}


void Utils::addReflectiveImagesAround(Mat image_warped, Mat& middle) {

	Mat  mirror_top, mirror_bottom;
	Mat left, right;

	//MIDDLE
	image_warped.copyTo(middle);
	flip(middle, mirror_top, 0);
	flip(middle, mirror_bottom, -1);
	flip(mirror_bottom, mirror_bottom, 1);
	vconcat(mirror_top, middle, middle);
	vconcat(middle, mirror_bottom, middle);

	//LEFT
	flip(image_warped, left, 1);

	flip(left, mirror_top, 0);
	flip(left, mirror_bottom, -1);
	flip(mirror_bottom, mirror_bottom, 1);
	vconcat(mirror_top, left, left);
	vconcat(left, mirror_bottom, left);

	//RIGHT
	flip(image_warped, right, 1);

	flip(right, mirror_top, 0);
	flip(right, mirror_bottom, -1);
	flip(mirror_bottom, mirror_bottom, 1);
	vconcat(mirror_top, right, right);
	vconcat(right, mirror_bottom, right);

	//All
	hconcat(left, middle, middle);
	hconcat(middle, right, middle);

	left.release();
	right.release();
	mirror_bottom.release();
	mirror_top.release();

}

Vec3b Utils::BilinearInterpolation(double new_x, double new_y, Mat img2) {
	Vec3b v;
	double intensity_left_right;
	double intensity_left_down;
	double intensity_result;
	for (int i = 0; i < 3; i++) {
		intensity_left_right = ((ceil(new_x) - new_x) / (ceil(new_x) - floor(new_x))) * img2.at<Vec3b>((int)floor(new_y), (int)floor(new_x))[i]
			+ ((new_x - floor(new_x)) / (ceil(new_x) - floor(new_x))) * img2.at<Vec3b>((int)floor(new_y), (int)ceil(new_x))[i];

		intensity_left_down = ((ceil(new_x) - new_x) / (ceil(new_x) - floor(new_x))) * img2.at<Vec3b>((int)ceil(new_y), (int)floor(new_x))[i]
			+ ((new_x - floor(new_x)) / (ceil(new_x) - floor(new_x))) * img2.at<Vec3b>((int)ceil(new_y), (int)ceil(new_x))[i];

		intensity_result = ((ceil(new_y) - new_y) / (ceil(new_y) - floor(new_y))) * intensity_left_right
			+ ((new_y - floor(new_y)) / (ceil(new_y) - floor(new_y))) * intensity_left_down;

		v[i] = (int)intensity_result;

	}

	return v;
}
