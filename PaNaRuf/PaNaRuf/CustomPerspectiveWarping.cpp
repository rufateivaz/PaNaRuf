#include "CustomPerspectiveWarping.h"

int CustomPerspectiveWarping::applyCustomPerspectiveWarping(vector<String> image_names) {

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

	// Homography matrices for each image relative to each other are stored in Hs vector.
	vector<Mat> Hs(images.size());
	// We set the middle image as static image. Therefore, we set its perspective transformation matrix is I.
	Hs[images.size() / 2] = Mat::eye(Size(3, 3), CV_64F); // I matrix.

	/*
		We choose middle image as static image (H = I). Starting from this image,
		using the pairwise relations (homographies) we estimate the 
		perspective transformation matrices for the other images relative to
		each other. Therefore, we multiply the homograpies nestedly with respect to
		images' relationships. We apply those operations until we find perspective
		transformation matrix for all images.
	*/
	for (int i = 0; i < images.size(); i++) {
		for (int j = 0; j < pairs.size(); j++) {
			if (Hs[pairs[j].getObj()].empty() && !Hs[pairs[j].getScene()].empty()) {
				Hs[pairs[j].getObj()] = Hs[pairs[j].getScene()] * pairs[j].getH();
			}
			if (Hs[pairs[j].getScene()].empty() && !Hs[pairs[j].getObj()].empty()) {
				Hs[pairs[j].getScene()] = Hs[pairs[j].getObj()] * pairs[j].getH().inv();
			}
		}
	}

	/*
		- warped_images : stores each warped image
		- warped_mask : stores each warped mask
		- corners : stores top-left points of each warped image
		- sizes : stores the image sizes of each warped image
	*/
	vector<Mat> warped_images;
	vector<Mat> warped_masks;
	vector<Point> corners;
	vector<Size> sizes;

	/*
		Applying  warping operations here...
	*/
	input_output.StartApplyingPerspectiveWarping();
	Warping(images, Hs, warped_images, warped_masks, corners, sizes);

	// Starts finding seams among the warped  images and stitchs them using multi-band blending.
	input_output.StartApplyingBlending();
	Mat result;
	blending.applyMultiBandBlending(warped_masks, warped_images, corners, sizes, result);

	// Writes the output image.
	input_output.prepareOutputImage(PERSPECTIVE, result);
	return 0;
}



void CustomPerspectiveWarping::Warping(vector<Mat> images, vector<Mat> Hs, vector<Mat>& warped_images, vector<Mat>& warped_masks, vector<Point>& corners, vector<Size>& sizes) {

	//Loop over each image and apply the following.
	for (int i = 0; i < images.size(); i++) {

		cout << "Warping " << (i + 1) << "/" << images.size() << endl;

		/*
			Create border-reflect effect around the images
			which we need for multi-band blending.
		*/
		Mat border_reflected_image;
		utils.addReflectiveImagesAround(images[i], border_reflected_image);
		Rect roi;
		try {
			// applying forward warping for i-th image.
			roi = forwardWarping(images[i], Hs[i]); //applies forward warping and finds size of the dst/warped image.
			if (!roi.empty() && roi.area() == 0)
				continue;
			else if (roi.empty())
				continue;
		}
		catch (exception e) {
			cout << "A problem occured. Probably, the area of the destination image is too big! (overflow/infinity)" << endl;
			continue;
		}
		/*
			After finding the top-left corner point of each warped (destination) image along with its size,
			we keep track of each of those data which we need in applying multi-band blending.
		*/
		corners.push_back(roi.tl()); // top-left corner points of each warped image.
		sizes.push_back(roi.size()); // size of each warped image.


		Mat warped_image, warped_mask; // destination warped image and mask

		/*
			Apply backward mapping.
		*/
		BackwardWarping(border_reflected_image, roi, warped_image, warped_mask, Hs[i]);

		// add each warped  images and corresponding masks to the vectors below.
		warped_images.push_back(warped_image);
		warped_masks.push_back(warped_mask);

		// free all of the created matrices.
		warped_image.release();
		warped_mask.release();
		border_reflected_image.release();

	}
}


Rect CustomPerspectiveWarping::forwardWarping(Mat image, Mat H) {
	int w = image.cols; // width of the source image
	int h = image.rows; // height of the source image
	Point2i topLeft = Point2i(INT_MAX, INT_MAX); // top-left point of the destination image
	Point2i bottomRight = Point2i(INT_MIN, INT_MIN); // bottom-right of the destination image

	/*
		Walk over each image points and apply the forward mapping to get
		the warped (destination) image points.
	*/
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {

			//2D Image points.
			Mat p2d = Mat(Size(1, 3), CV_64F);
			p2d.at<double>(0, 0) = double(i);
			p2d.at<double>(1, 0) = double(j);
			p2d.at<double>(2, 0) = 1.0;

			//warping perspective
			Mat point2d = H * p2d;

			//warped (destination) image point.
			point2d = point2d / point2d.at<double>(2, 0);

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
		return Rect(Point2i(0, 0), Point2i(0, 0));
	
	return Rect(topLeft, bottomRight);
}

void CustomPerspectiveWarping::BackwardWarping(Mat image, Rect roi, Mat& warped_image, Mat& warped_mask, Mat H) {
	int ws = image.cols; // width of source image.
	int hs = image.rows; // height of source image.
	int wd = roi.br().x - roi.tl().x; // width of warped (destination) image.
	int hd = roi.br().y - roi.tl().y; // height of warped (destination) image.

	cout << wd << "x" << hd << endl;

	// offset is used to shift the warped (dest) image point to actual point in x-y image coordinates. 
	Point2d offset = 0.5 * Point2d((roi.tl() + roi.br())) - Point2d(double(wd) * 0.5, double(hd) * 0.5);

	// warped_image is destination (warped) image
	warped_image = Mat(Size(wd, hd), CV_8UC3, Scalar(0));

	// warped_mask is destination (warped) mask
	warped_mask = Mat(Size(wd, hd), CV_8U, Scalar(0));

	// we use omp to paralelize the pixelwise operations via multithreading  
#pragma omp parallel for schedule(static)
	for (int x = 0; x < wd; x++) {
		for (int y = 0; y < hd; y++) {

			//warped (dest) image point
			Mat p2d = Mat(Size(1, 3), CV_64F);
			p2d.at<double>(0, 0) = (x + offset.x);
			p2d.at<double>(1, 0) = (y + offset.y);
			p2d.at<double>(2, 0) = 1.0;

			//inverse perspective transforming (source image point)
			Mat point2d = H.inv() * p2d;

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

			/* Below, we check if the requested pixel value is within the src image borders,
				* If yes, we apply bilinear-interpolation method to color each pixel
				  of destination image. And we give white color to mask.
				* If no,  we ignore the current pixel value of destination image and
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
