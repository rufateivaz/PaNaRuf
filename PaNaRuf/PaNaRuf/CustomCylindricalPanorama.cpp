#include "CustomCylindricalPanorama.h"

int CustomCylindricalPanorama::applyCustomCylindricalWarping(vector<String> image_names) {

	// The images are read and assigned to the vector below
	vector<Mat> images(image_names.size());
	
	// Takes the return value from addInputImages function.
	int assignId = utils.addInputImages(image_names, images);

	/*
		If all images are added successfully to "images" vector, then it returns -1.
		Otherwise, the index of the problematic image is returned to print it out.
	*/
	if (assignId != -1) {
		input_output.readingError(image_names[assignId]);
		return -1;
	}

	// Checking if there are enough images to apply panorama.  
	if (images.size() < 2) {
		input_output.NoEnoughImagesException();
		return -1;
	}

	/*
		Finds all of the pairwise relations among the images which will be stitched.
	*/
	input_output.StartPairwiseMatches();
	vector<PairwiseMatches> pairs;
	relationFinder.findRelationsAmongImages(images, pairs);


	// Checking if there are enough images to apply panorama.  
	if (images.size() < 2) {
		input_output.NoEnoughPairedImagesException();
		return -1;
	}

	/*
		Start to estimate the camera (intrinsic,extrinsic) parameters.
	*/

	CustomCameraParameterEstimation estimator; 
	vector<double> focals; // gets all possible estimated focals.

	// Add all possible focal length values estimated from homographies to the list "focals" (based on Szeliski p.57)
	estimator.EstimateFocals(pairs, focals);

	// The corresponding camera parameters of each taken image is stored in the vector below.
	vector<CameraParameters> cameraParams(images.size());

	//initialize final focal.
	double focal = 0;

	/*
		* If we have estimated enough focals from homographies,
		  then we choose the median among them. (based on Szeliski book page 57)

		* Else, by default we choose the focal length of the camera matrix using image widths and heights.
	*/
	if (focals.size() >= pairs.size()) {
		focal = estimator.getFocalByMedian(focals);
	}
	else {
		focal = estimator.getFocalByDefault(images);
	}

	/*
		Set the intrinsic (focals,principal points) parameters of all cameras.
	*/
	estimator.setCalibrationMatrices(images, cameraParams, focal);

	/*
		Using homographies and calibration (K) matrices, we estimate each Rotation matrices for each camera.
	*/
	estimator.setRotationMatrices(images, pairs, cameraParams);


	/*
		prints the camera parameters including Rotation (extrinsic) and Calibration (intrinsic) parameters.
	*/
	input_output.printCameraParameters(cameraParams);

	/*
		- warped_images : stores each cylindrical image
		- warped_mask : stores each cylindrical mask
		- corners : stores top-left points of each cylindrical image
		- sizes : stores the image sizes of each cylindrical image
	*/
	vector<Mat> warped_images;
	vector<Mat> warped_masks;
	vector<Point> corners;
	vector<Size> sizes;

	/*
		Applying cylindrical warping operations here...
	*/
	input_output.StartApplyingCylindricalWarping();
	Warping(images, cameraParams, warped_images, warped_masks, corners, sizes); 

	// Starts finding seams among the warped cylindrical images and stitchs them using multi-band blending.
	input_output.StartApplyingBlending();
	Mat result;
	blending.applyMultiBandBlending(warped_masks, warped_images, corners, sizes, result);

	// Writes the output image.
	input_output.prepareOutputImage(CYLINDRICAL, result);
	return 0;
}



void CustomCylindricalPanorama::Warping(vector<Mat> images, vector<CameraParameters> cameraParams, vector<Mat>& warped_images, vector<Mat>& warped_masks, vector<Point>& corners, vector<Size>& sizes) {

	//Loop over each source image and apply the following.
	for (int i = 0; i < images.size(); i++) {

		cout << "Warping " << (i + 1) << "/" << images.size() << endl;

		/*
			Creating border-reflect effect around the images
			which we need for multi-band blending.
		*/
		Mat border_reflected_image;
		utils.addReflectiveImagesAround(images[i], border_reflected_image);

		// Applying forward warping for i-th image.
		Rect roi = forwardWarping(images[i], cameraParams[i].getK(), cameraParams[i].getR()); //applies forward warping and finds size of the dst/warped image.
		if (!roi.empty() && roi.area() == 0)
			continue;
		else
			if (roi.empty())
				continue;

		/*
			After finding the top-left corner point of each cylindrical image along with its size,
			we keep track of each of those data which we need in applying multi-band blending.
		*/
		corners.push_back(roi.tl()); // top-left corner points of each cylindrical image.
		sizes.push_back(roi.size()); // size of each cylindrical image.


		Mat warped_image, warped_mask; // destination (cylindrical) image and destination mask
		
		/*
			Apply backward mapping for i-th cylindrical image.
		*/
		BackwardWarping(border_reflected_image, roi, warped_image, warped_mask, cameraParams[i].getK(), cameraParams[i].getR());

		// add each warped cylindrical images and corresponding masks to the vectors below.
		warped_images.push_back(warped_image);
		warped_masks.push_back(warped_mask);

		// free all of the created matrices.
		warped_image.release();
		warped_mask.release();
		border_reflected_image.release();

	}
}

	
Rect CustomCylindricalPanorama::forwardWarping(Mat image, Mat K, Mat R) {
	int w = image.cols; // width of the source image
	int h = image.rows; // height of the source image
	Point2i topLeft = Point2i(INT_MAX, INT_MAX); // top-left point of the destination image
	Point2i bottomRight = Point2i(INT_MIN, INT_MIN); // bottom-right of the destination image

	/*
		Instead of multiplying R^-1 by K^-1 at each iteration, we multiply it beforehand
		to speed-up the program.
	*/
	Mat Rinv_Kinv = R.inv() * K.inv();

	/*
		Walk over each source image points and apply the forward mapping to get
		the destination (cylindrical) image points.
	*/
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {

			//2D Image points.
			Mat p2d = Mat(Size(1, 3), CV_64F);
			p2d.at<double>(0, 0) = double(i);
			p2d.at<double>(1, 0) = double(j);
			p2d.at<double>(2, 0) = 1.0;

			//2D-3D un-projection and rotation.
			Mat point3d = Rinv_Kinv * p2d;

			//scale value is sqrt (X^2 + Z^2)
			double scale_to_cyl = sqrt(pow(point3d.at<double>(0, 0), 2) + pow(point3d.at<double>(2, 0), 2));

			//3D to unit-cylinder coordinates.
			Mat point3d_cyl = point3d / scale_to_cyl;

			//Longitude and Latitude (OR theta and phi) values 
			Mat cyl = Mat(Size(1, 3), CV_64F);
			cyl.at<double>(0, 0) = atan2(point3d_cyl.at<double>(0, 0), point3d_cyl.at<double>(2, 0));
			cyl.at<double>(1, 0) = point3d_cyl.at<double>(1, 0);
			cyl.at<double>(2, 0) = 1.0;

			//to 2D cylindrical image points.
			Mat point2d = Mat(Size(1, 3), CV_64F);;
			point2d.at<double>(0, 0) = K.at<double>(0, 0) * cyl.at<double>(0, 0);
			point2d.at<double>(1, 0) = K.at<double>(1, 1) * cyl.at<double>(1, 0);

			/*
				The following operations are applied to get min and maximum destination image points,
				where min is top-left and max is bottom-right.
			*/
			if (topLeft.x > (int)round(point2d.at<double>(0, 0)))
				topLeft.x = (int)round(point2d.at<double>(0, 0));
			if (topLeft.y > (int)round(point2d.at<double>(1, 0)))
				topLeft.y = (int)round(point2d.at<double>(1, 0));
			if (bottomRight.x < (int)round(point2d.at<double>(0, 0)))
				bottomRight.x = (int)round(point2d.at<double>(0, 0));
			if (bottomRight.y < (int)round(point2d.at<double>(1, 0)))
				bottomRight.y = (int)round(point2d.at<double>(1, 0));
		}

	}

	if (topLeft.x == INT_MAX || topLeft.y == INT_MAX || bottomRight.x == INT_MIN || bottomRight.y == INT_MIN)
		return Rect(Point(0, 0), Point(0, 0));
	return Rect(topLeft, bottomRight);
}

void CustomCylindricalPanorama::BackwardWarping(Mat image, Rect roi, Mat& warped_image, Mat& warped_mask, Mat K, Mat R) {
	int ws = image.cols; // width of source image.
	int hs = image.rows; // height of source image.
	int wd = roi.br().x - roi.tl().x; // width of destination (cylindrical) image.
	int hd = roi.br().y - roi.tl().y; // height of destination (cylindrical) image.

	cout << wd << "x" << hd << endl;

	// offset is used to shift the cylindrical image point to actual point in x-y image coordinates. 
	Point2d offset = 0.5 * Point2d((roi.tl() + roi.br())) - Point2d(double(wd) * 0.5, double(hd) * 0.5);

	// warped_image is destination (cylindrical) image
	warped_image = Mat(Size(wd, hd), CV_8UC3, Scalar(0));

	// warped_mask is destination (cylindrical) mask
	warped_mask = Mat(Size(wd, hd), CV_8U, Scalar(0));

	/*
		Instead of multiplying K by R at each step of iteration, we multiply them here.
		(this improves the overall complexity of mapping operations)
	*/
	Mat KR = K * R;

// we use omp to paralelize the pixelwise operations via multithreading  
#pragma omp parallel for
	for (int x = 0; x < wd; x++) {
		for (int y = 0; y < hd; y++) {
			// un-unproject from 2D cylindrical image coordinate to get theta and phi respectively.
			Mat cyl = Mat(Size(1, 3), CV_64F);
			cyl.at<double>(0, 0) = (x + offset.x) / K.at<double>(0, 0);
			cyl.at<double>(1, 0) = (y + offset.y) / K.at<double>(1, 1);
			cyl.at<double>(2, 0) = 1.0;

			// Obtain unit-cylinder coordinates from theta and phi
			Mat point3d = Mat(Size(1, 3), CV_64F);
			point3d.at<double>(0, 0) = sin(cyl.at<double>(0, 0));
			point3d.at<double>(1, 0) = cyl.at<double>(1, 0);
			point3d.at<double>(2, 0) = cos(cyl.at<double>(0, 0));

			// pass to 3D coordinates. We use fabs since cosine can also be negative.
			point3d = point3d / fabs(point3d.at<double>(2, 0));

			// Rotate in 3D and project to 2D source image coordinate.
			Mat point2d = KR * point3d;

			// If the point is beyond camera, do not take it.
			if (point2d.at<double>(2, 0) <= 0)
				continue;

			// normalize to [x,y,1]
			point2d = point2d / point2d.at<double>(2, 0);

			// corresponding point
			Point2d pixel = Point2d(point2d.at<double>(0, 0), point2d.at<double>(1, 0));

			/* 
				Since we added self-reflected images around the source image. We
				need to shift the point from top left to middle.
			*/
			pixel.x += ((double)ws / 2.0 - (double)ws / 6.0);
			pixel.y += ((double)hs / 2.0 - (double)hs / 6.0);

			/* Below, we check if the requested pixel value is within the source image borders,
				* If yes, we apply bilinear-interpolation method to color each pixel
				  of destination (cylindrical) image. And, we give white color to mask.
				* If no,  we ignore the current pixel value of destination (cylindrical) image and
				  we give black color to mask.
			*/
			if (pixel.x > 1 && pixel.x < (double(ws) - 1.0) &&
				pixel.y > 1 && pixel.y < (double(hs) - 1.0) &&
				floor(pixel.x) != ceil(pixel.x) &&
				floor(pixel.y) != ceil(pixel.y)) {

				warped_image.at<Vec3b>(y, x) = utils.BilinearInterpolation(pixel.x, pixel.y, image);
			}

			else if (round(pixel.x) >= 0 && round(pixel.x) < ws &&
				round(pixel.y) >= 0 && round(pixel.y) < hs) {

				warped_image.at<Vec3b>(y, x) = image.at<Vec3b>((int)round(pixel.y), (int)round(pixel.x));
			}


			if (pixel.x > (double(ws) / 3.0) && pixel.x < ((double(ws) / 3.0) * 2.0 - 1.0) &&
				pixel.y >(double(hs) / 3.0) && pixel.y < ((double(hs) / 3.0) * 2.0 - 1.0)) {

				warped_mask.at<uchar>(y, x) = 255;
			}
			else {
				warped_mask.at<uchar>(y, x) = 0;
			}
		}

	}
}

