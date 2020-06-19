#include "Blending.h"

void Blending::applyMultiBandBlending(vector<Mat> warped_masks, vector<Mat> warped_images,
	vector<Point> corners, vector<Size> sizes, Mat& result
) {

	/*
		This function finds seams among the warped images.
		Then, it stitches those images using multi-band blending algorithm.

		Note: Thanks to OpenCV, we used its open-source  methods and used them in the function below
			  and arranged for our software.
	*/

	vector<UMat> umats_masks(warped_masks.size());
	vector<UMat> umats_images(warped_images.size());

	for (int i = 0; i < warped_masks.size(); i++) {

		warped_masks[i].copyTo(umats_masks[i]);
		warped_images[i].copyTo(umats_images[i]);

		umats_images[i].convertTo(umats_images[i], CV_32F);

	}

	cout << "Finding seams..." << endl;
	
	/*
		VoronoiSeamFinder is faster than GraphCutSeamFinder but 
		the quality of the final image is vice versa.

		So, depending on the intention, one can comment/un-comment any of them below,
		to choose for finding the seams among the warped images.
	*/

	//Ptr<SeamFinder> seam_finder = makePtr<VoronoiSeamFinder>();

	Ptr<SeamFinder> seam_finder =  makePtr<GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR); 

	seam_finder->find(umats_images, corners, umats_masks); //this estimates the seams using corners and images_warped and assigning to masks_warped

	cout << "Seams found" << endl;
	for (int i = 0; i < warped_masks.size(); i++) {

		umats_images[i].convertTo(umats_images[i], CV_8UC3);

		Mat dilated_mask, seam_mask;
		dilate(umats_masks[i], dilated_mask, Mat()); // we are enlarging warped masks and store in dilated mask
		resize(dilated_mask, seam_mask, warped_masks[i].size(), 0, 0, INTER_LINEAR_EXACT); //resizing dilated mask by warped mask size, store to seam mask
		warped_masks[i] = seam_mask & warped_masks[i];
	}


	Ptr<Blender> blender;
	for (int i = 0; i < warped_images.size(); i++) {

		if (i == 0) // only in the first iteration we need to initialize our multi-band blender
		{
			float blend_width = sqrt((float)(resultRoi(corners, sizes).size().area())) * 5 / 100.f; // finding blend width
			cout << "Blend width = " << blend_width << endl;
			if (blend_width < 1.f) // check if the blend width above is less than 1 
				blender = Blender::createDefault(Blender::NO, false); // if it is less than 1 then we create blender with type NO which means simple blender putting over another. We do not try gpu so it is false.
			else
			{
				blender = Blender::createDefault(Blender::MULTI_BAND, false); // if it is greater/equal than 1, then we create blender with type MULTO_BAND. We do not try gpu that's why it is false.
				MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get()); // We use multi-band blender algorithm for blending
				mb->setNumBands((int)(ceil(log(blend_width) / log(2.)) - 1.)); // calculation of number of bands and setting to our mb object
			}
			blender->prepare(corners, sizes); //Preparation to blending

		}

		warped_images[i].convertTo(warped_images[i], CV_16S); //Change type of warped image from 8bit unsigned int to 16 bit signed int 

		blender->feed(warped_images[i], warped_masks[i], corners[i]); 
	}


	Mat result_mask;
	blender->blend(result, result_mask); // Blending and storing the final panoramic image in "result".
}