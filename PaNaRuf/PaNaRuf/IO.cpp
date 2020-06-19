#include "IO.h"



void IO :: readImageNames(vector<string>& image_names, char* argv[]) {
	ifstream infile(argv[1]);
	string image_name;
	while (infile >> image_name)
	{
		cout << "Reading image : " << image_name << endl;
		image_names.push_back(image_name);
	}
}

void IO::writeFieldOfViewError() {
	cout << "Please write the horizontal field of view in argv[2] and/or the vertical field of view in argv[3]." << endl;
}

void IO::writeTypeOfPanoError() {
	cout << "Please write the type of panorama in argv[1] (e.g  p - perspective, c - cylindrical, s - spherical)." << endl;
}

void IO::provideCorrectInputError() {
	cout << "Please write the input correctly." << endl
		<< "arg1: */*.txt file containing the image files." << endl
		<< "arg2: Type of panorama (e.g  -p - perspective, -c - cylindrical, -s - spherical)." << endl
		<< "arg3: If the type of panorama is -s - spherical then also write horizontal and vertical field of view (e.g hfov = 180  vfov = 180)." << endl;
}


void IO::readingError(string image_name) {
	cout << "The image with name " << image_name << " gives empty error.Please check the reason and try again." << endl;
}

void IO::NoEnoughImagesException() {
	cout << "There should be at least 2 images to apply stitching algorithm!" << endl;
}

void IO::NoEnoughPairedImagesException() {
	cout << "There is no images with enough good matches to apply stitching algorithm" << endl;
}

void IO::StartPairwiseMatches() {
	cout << "Finding pairwise matches .... " << endl;
}

void IO::StartGettingRectilinearImages() {
	cout << "Getting rectilinear images from given each fisheye image .... " << endl;
}

void IO::StartFindingRelations() {
	cout << "Finding relations among image sets .... " << endl;
}

void IO::StartPairwiseHomography() {
	cout << endl;
	cout << "Computing homography matrices between each pairs .... " << endl;
}

void IO::StartConnectingImagePairs() {
	cout << endl;
	cout << "Finding image pairs .... " << endl;
}

void IO::ShowConnectedImagePairs(vector<PairwiseMatches> pairs, vector<string> image_names) {
	for (int i = 0; i < pairs.size(); i++) {
		cout << image_names[pairs[i].getObj()] << " is pair with " << image_names[pairs[i].getScene()] << endl;
	}

}

void IO::StartFindingTransformationMatrix() {
	cout << endl;
	cout << "Finding transformation matrix for each image separately ... " << endl;
}

void IO::printTransformationMatrix(vector<Mat> Ms) {
	cout << endl;
	for (int i = 0; i < Ms.size(); i++) {
		cout << "M" << i << " = " << Ms[i] << endl;
	}
}

void IO::StartFindingCameraParameters() {
	cout << endl;
	cout << "Finding camera parameters for each image separately ... " << endl;
}

void IO::printCameraParameters(vector<CameraParameters> cameraParams) {
	cout << endl;
	for (int i = 0; i < cameraParams.size(); i++) {
		cout << "K" << i << " = " << cameraParams[i].getK() << endl;
		cout << "R" << i << " = " << cameraParams[i].getR() << endl;
		cout << "det = " << determinant(cameraParams[i].getR()) << endl;
	}
}

void IO::StartApplyingCylindricalWarping() {
	cout << endl;
	cout << "Applying cylindrical warping for each image ... " << endl;
}

void IO::StartApplyingPerspectiveWarping() {
	cout << endl;
	cout << "Applying perspective warping for each image ... " << endl;
}

void IO::StartApplyingSphericalWarping() {
	cout << endl;
	cout << "Applying spherical warping for each image ... " << endl;
}

void IO::StartApplyingBlending() {
	cout << endl;
	cout << "Applying blending operations ... " << endl;
}

void IO::prepareOutputImage(PanoramaType panoType, Mat result) {
	switch (panoType) {
	case(PERSPECTIVE):
		imwrite("outputs/perspective/result.png", result);
		break;
	case(CYLINDRICAL):
		imwrite("outputs/cylindrical/result.png", result);
		break;
	case(SPHERICAL):
		imwrite("outputs/spherical/result.png", result);
		break;
	default:
		break;
	}
}