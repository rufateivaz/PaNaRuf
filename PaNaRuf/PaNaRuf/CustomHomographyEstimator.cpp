#include "CustomHomographyEstimator.h"

void CustomHomographyEstimator :: Normalization(vector<Point2d>& points, Mat& T) {

/*
Based on Hartley and Zisserman (Multiple View Geometry) the normalization is to
compute a similarity transform T that takes "points" to a new set
of points "p_prime" such that the centroid of "p_prime" is the coordinate
origin and their average distance from the origin is sqrt(2)
*/

	/*
		Finding centroid of the points in "points" vector.
	*/
	Point2d centroid = Point2d(0, 0);

	for (int i = 0; i < points.size(); i++)
		centroid = centroid + points[i];

	centroid = centroid / double(points.size());

	/*
		Finds the average distance from centroid to each points in "points" vector.
	*/
	double avg_dist = 0;
	for(int i=0; i < points.size() ; i++) 
		avg_dist += sqrt(pow((points[i].x - centroid.x), 2) + pow((points[i].y - centroid.y), 2));
	avg_dist = avg_dist / double(points.size());

	/*
		Similarity transform T
	*/
	T = Mat::eye(Size(3, 3), CV_64F);
	T.at<double>(0, 0) = sqrt(2.0) / avg_dist;
	T.at<double>(0, 2) = -(sqrt(2.0) / avg_dist) * centroid.x;
	T.at<double>(1, 1) = sqrt(2.0) / avg_dist;
	T.at<double>(1, 2) = -(sqrt(2.0) / avg_dist) * centroid.y;
	T.at<double>(2, 2) = 1.0;

	/*
		taking "points" to the new set of points
	*/
	for (int i = 0; i < points.size(); i++) {
		Mat p = Mat(Size(1, 3), CV_64F);
		p.at<double>(0, 0) = points[i].x;
		p.at<double>(1, 0) = points[i].y;
		p.at<double>(2, 0) = 1.0;

		Mat p_prime = T * p;

		p_prime = p_prime / p_prime.at<double>(2, 0);

		points[i].x = p_prime.at<double>(0, 0);
		points[i].y = p_prime.at<double>(1, 0);
	}
}

bool CustomHomographyEstimator::isColinear(vector<Point2d> p) {


	/*
		We take three points combinations and check if the area of the triangle is 0
		If yes, then the points lie on the same line: FALSE:-> co-linearity.
		If no, then TRUE:-> no co-linearity.
		[Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By)] / 2 ?= 0
	*/

	//Check first 3 points
	if (fabs((p[0].x * (p[1].y - p[2].y) + p[1].x * (p[2].y - p[0].y) + p[2].x * (p[0].y - p[1].y)) / 2) < 0.00001)
		return true;

	//Check second 3 points
	if (fabs((p[1].x * (p[2].y - p[3].y) + p[2].x * (p[3].y - p[1].y) + p[3].x * (p[1].y - p[2].y)) / 2) < 0.00001)
		return true;

	//Check third 3 points
	if (fabs((p[0].x * (p[1].y - p[3].y) + p[1].x * (p[3].y - p[0].y) + p[3].x * (p[0].y - p[1].y)) / 2) < 0.00001)
		return true;

	return false;
}

bool CustomHomographyEstimator::chosenSamePoint(vector<int> indexes) {


	/*
		We randomly choose 4 different points to apply DLT.
		Since they are chosen randomly, we do not know
		if there exist repeating (same) index of points in the list.
		If yes, then we can not apply DLT algorithm since 
		the number of points will be < 4. (should be >= 4)

		Therefore, with this function we check if we have 
		randomly chosen any repeated index of points or not. 
		
		if yes: false
		otherwise: true
	*/

	for (int i = 0; i < indexes.size(); i++) {
		for (int j = i+1; j < indexes.size(); j++) {
			if (indexes[i] == indexes[j])
				return true;
		}
	}

	return false;
}

double CustomHomographyEstimator::distanceThreshold(vector<double>& forward_errors, vector<double>& backward_errors, Mat H , vector<Point2d> obj, vector<Point2d> scene) {
	/*
		This function calculates distanceThreshold t where:
				t = sqrt(5.99) * sigma and sigma is standard deviation.
	*/
	
	// The number of good matches between overlapping images.
	int numberOfMatches = (int)obj.size();
	//Sum of the errors derived from the forward perspective transformation
	double sumOfForwardErrors = 0;
	//Sum of the errors derived from the backward perspective transformation
	double sumOfBackwardErrors = 0;


	for (int i = 0; i < numberOfMatches; i++) {
		// forward transformation
		Mat p_obj = Mat(Size(1, 3), CV_64F);
		p_obj.at<double>(0, 0) = obj[i].x;
		p_obj.at<double>(1, 0) = obj[i].y;
		p_obj.at<double>(2, 0) = 1.0;

		Mat p_scene = H * p_obj;
		p_scene = p_scene / p_scene.at<double>(2, 0);
		
		/* 
			obtaining :
			* Error while forward transformation.
			* Sums all of the forward transformation errors.

			* Stores each forward transformation error in forward errors
		*/
		double forward_error = sqrt(pow(p_scene.at<double>(0, 0) - scene[i].x, 2.0) + pow(p_scene.at<double>(1, 0) - scene[i].y, 2.0));
		sumOfForwardErrors += forward_error;
		forward_errors.push_back(forward_error);


		//backward transformation
		p_scene.at<double>(0, 0) = scene[i].x;
		p_scene.at<double>(1, 0) = scene[i].y;
		p_scene.at<double>(2, 0) = 1.0;

		p_obj = H.inv() * p_scene;
		p_obj = p_obj / p_obj.at<double>(2, 0);
		
		/*
			obtaining :
			* Error while backward transformation.
			* Sums all of the backward transformation errors.

			* Stores each backward transformation error in forward errors
		*/
		double backward_error = sqrt(pow(p_obj.at<double>(0, 0) - obj[i].x, 2.0) + pow(p_obj.at<double>(1, 0) - obj[i].y, 2.0));
		sumOfBackwardErrors += backward_error;
		backward_errors.push_back(backward_error);
	}

	//calculates standard deviation.
		double totalErrors = sumOfForwardErrors + sumOfBackwardErrors;
		double sigma = sqrt(totalErrors / (2 * double(numberOfMatches)));
	//returns distanceThreshold.
	return  sqrt(5.99) * sigma;
}



void CustomHomographyEstimator::computeInliers(vector<Point2d> obj, vector<Point2d> scene, Mat H, vector<int>& inliers, int& numberOfInliers) {
	/*
		This function computes inliers and the number of inliers that holds the inequality:
			d < sqrt(5.99) * sigma (where sigma is standard deviation)
	*/
	
	// forward and backward errors list.
	vector<double> fowardErrors, backwardErrors;
	
	//calculate distance threshold and store each of forward and backward errors.
	double t = distanceThreshold(fowardErrors,backwardErrors, H, obj, scene);
	
	//the number of good matches.
	int numberOfMatches = (int)obj.size();

	for (int i = 0; i < numberOfMatches; i++) {
		/* 
			If both are less than distance threshold. Then the common i-th element 
			is inlier and consequently the numberOfInliers is incremented.
		*/
		if (fowardErrors[i] < t && backwardErrors[i] < t) {
			numberOfInliers++;
			inliers[i] = 1;
		}
	}
}


void CustomHomographyEstimator::DirectLinearTransform(vector< Point2d > obj, vector< Point2d > scene, Mat& H) {
	/*
		This function is used to estimate homography for every N >= 4 points combinations as below:
		* Using Singular Value Decomposition, solve Ah = 0 and assign the last row of Vt where U*W*Vt = SVD(A) to "h" vector.
		* Construct H (3x3) homography matrix from found "h" vector. 
	*/

	/*
		Normalize Dlt:
		* find similarity transformation matrices t_obj and t_scene.
		* using them transform obj and scene points respectively.
	*/
	Mat t_obj, t_scene;
	Normalization(obj, t_obj);
	Normalization(scene, t_scene);

	/*
		Declare A as 9x2N matrix.
		Fill its values based on x' = H*x
	*/
	int N = (int)obj.size();
	Mat A(Size(9, N*2), CV_64F, Scalar(0));
	for (int i = 0; i < N; i++) {
		A.at<double>(i * 2, 3) = (-1.0) * obj[i].x;
		A.at<double>(i * 2, 4) = (-1.0) * obj[i].y;
		A.at<double>(i * 2, 5) = -1.0;
		A.at<double>(i * 2, 6) = scene[i].y * obj[i].x;
		A.at<double>(i * 2, 7) = scene[i].y * obj[i].y;
		A.at<double>(i * 2, 8) = scene[i].y;

		A.at<double>(i * 2 + 1, 0) = obj[i].x;
		A.at<double>(i * 2 + 1, 1) = obj[i].y;
		A.at<double>(i * 2 + 1, 2) = 1.0;
		A.at<double>(i * 2 + 1, 6) = (-1.0) * scene[i].x * obj[i].x;
		A.at<double>(i * 2 + 1, 7) = (-1.0) * scene[i].x * obj[i].y;
		A.at<double>(i * 2 + 1, 8) = (-1.0) * scene[i].x;

	}
	/*
		Use singular value decomposition to find h from Ah = 0
	*/
	Mat W, U, Vt;
	Mat h(Size(9, 1), CV_64F);
	SVD::compute(A, W, U, Vt, SVD::FULL_UV);
	h = Vt.row(8);

	/*
		Construct H (homography) matrix from found "h" vector
	*/
	H = Mat(Size(3, 3), CV_64F);
	int index = 0;
	for (int k = 0; k < 9; k++) {
		H.at<double>(index, k % 3) = h.at<double>(0, k);
		if ((k + 1) % 3 == 0) {
			index++;
		}
	}
	//normalize homography.
	H = H / H.at<double>(2, 2);

	// Denormalize for getting H.
	H = t_scene.inv() * H * t_obj;

	//free unnecessary matrices.
	t_scene.release();
	t_obj.release();
}



void CustomHomographyEstimator::EstimateHomography(vector<Point2d> obj, vector<Point2d> scene, Mat& H_best, bool& isHfound, int& max_inliers) {
	
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

	int N = 1000; //number of iteration
	double p = 0.9949; //probability value (generally it is set to 0.99)
	int numberOfMatches = (int)obj.size(); //number of matching points between two images.
	

	isHfound = false;

	/*
		Randomly selected 4 sample points are stored in "points1" and "points2".
		Indexes of the selected points are stored in "indexes".
	*/
	vector<Point2d> points1(4); 
	vector<Point2d> points2(4);
	vector<int> indexes(4);

	// The best (updated iteratively) is stored in "inliers"
	vector<int> inliers_best(numberOfMatches);

	// Loop over N
	for (int i = 0; i < N; i++) {

		// Stores the inliers obtained at each iteration
		vector<int> inliers_current(numberOfMatches); 
		
		//Randomly selecting indexes of the correspondences.
		for (int j = 0; j < 4; j++) {
			int index = rand() % numberOfMatches;
			points1[j].x = obj[index].x;
			points1[j].y = obj[index].y;
			points2[j].x = scene[index].x;
			points2[j].y = scene[index].y;
			indexes[j] = index;
		}
		
		/*
			Checking if the chosen points are good enough
			* If no, ignore and continue to loop.
			
			Case1:
			Colinearity means "at least 3 chosen points are on the same line".
			In this case, it is impossible to apply DLT.

			Case2:
			The chosen points should be different. We should avoid choosing same points,
			because we need 4 different points to apply DLT.

		*/
		if (isColinear(points1) || isColinear(points2) || chosenSamePoint(indexes))
			continue;


		// Estimate the current homography via Direct Linear Transform method.
		Mat H;
		DirectLinearTransform(points1, points2, H);
		

		// Compute the current inliers via current Homography (H) matrix.
		int numinlier = 0;
		computeInliers(obj, scene, H, inliers_current, numinlier);
		
		// Choose the better homography which has more inliers.
		if (max_inliers < numinlier){
			max_inliers = numinlier; 
			H.copyTo(H_best);
			inliers_best.assign(inliers_current.begin(), inliers_current.end());
			isHfound = true;

			/* Update N. 
				e is the probability that a sample correspondence is an outlier 
			*/
			double e = 1 - (double)numinlier / (double)numberOfMatches;
			N = (int)round((log(1 - p) / log(1 - pow(1 - e, 4))));
		}

		//Free the un-necessary matrix and vectors.
		inliers_current.clear();
		H.release();
	}

	/*
		If we have estimated best homography, then we recompute H from the inliers points.
		(based on Elan Dubrofsky p.21)
	*/
	if (!H_best.empty()) {
		vector<Point2d> final_obj, final_scene;
		for (int i = 0; i < inliers_best.size(); i++) {
			if (inliers_best[i] == 1) {
				final_obj.push_back(obj[i]);
				final_scene.push_back(scene[i]);
			}
		}
		DirectLinearTransform(final_obj, final_scene, H_best);
	}
}