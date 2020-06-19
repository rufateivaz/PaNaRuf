#include <iostream>
#include "CustomRelationFinder.h"
#include <omp.h>
void CustomRelationFinder  :: findRelationsAmongImageSets(vector<vector<Mat>>& rectImagesSet, vector<vector<CameraParameters>>& rectCamerasSet , vector<String> image_names) {

    ComputeFeatures computeFeatures; // extracts features of an image
    vector<PairwiseMatches> all_pairs; // keeps track of all pairs of images.

     /*
        The nested loop below searches for possible relationships among the image sets.
     */

    for (int i = 0; i < rectImagesSet.size(); i++) {
        for (int j = i + 1; j < rectImagesSet.size(); j++) {
            //definition of lock which handles critical section problem for multi-threading
            omp_lock_t writelock;
            omp_init_lock(&writelock);
#pragma omp parallel for
            for (int ii = 0; ii < rectImagesSet[i].size(); ii++) {
                for (int jj = 0; jj < rectImagesSet[j].size(); jj++) {

                    // Define obj, scene such that scene  = H * scene;
                    vector<Point2d> obj, scene;

                    /*
                        Get the good matching points between ii-th and jj-th images and store their relationship
                        data in customly declared "PairwiseMatches" object
                    */
                    int numberOfGoodMatches = computeFeatures.imageFeatureComputer(rectImagesSet[j][jj], rectImagesSet[i][ii], obj, scene);
                    PairwiseMatches pm(j * 100 + jj, i * 100 + ii, obj, scene, numberOfGoodMatches);
                    
                    // We assume that there should be at least 8 good matches between ii-th and jj-th images to continue.
                    if (obj.size() >= 8) {

                        // Computes the homography.
                        pm.computeH();
                        
                        /*
                            If the homography between ii-th and jj-th images exists and
                            If this homography is really good enough, then
                            we can add the relation between those images to all_pairs vector.
                        */
                        if (pm.isHomographyFound() && pm.niceHomography()) {
                            PairwiseMatches pm_inv(i * 100 + ii, j * 100 + jj, obj, scene, numberOfGoodMatches);
                            pm_inv.setH(pm.getH().inv());
                            //only one thread can process the following at a time.
                            omp_set_lock(&writelock);
                                all_pairs.push_back(pm);
                                all_pairs.push_back(pm_inv);
                            omp_unset_lock(&writelock);

                        }
                    }
                }
            }
        }
    }


    /*
        Below what we are doing are :
        * Choose the best and final pairs among the image sets.
        * Find the relative rotation matrix and update all rotation matrices in the same set accordingly.
    */
    vector<bool> flags(rectImagesSet.size());
    flags[0] = true;
    for (int k = 1; k < flags.size(); k++) {
        int index = -1;
        int max = INT_MIN;
        for (int i = 0; i < flags.size(); i++) {
            if (!flags[i]) {
                for (int j = 0; j < flags.size(); j++) {
                    if (flags[j]) {
                        for (int p = 0; p < all_pairs.size(); p++) {
                            /*
                                searching best pair for flagged image set(s).
                                the one with maximum number of inliers is the best.
                            */
                            if (all_pairs[p].getObj() / 100 == i && all_pairs[p].getScene() / 100 == j && all_pairs[p].getNumberOfInliers() > max) {
                                max = all_pairs[p].getNumberOfInliers();
                                index = p;
                            }
                        }
                    }
                }
            }
        }

        if (index == -1) // No/not good relations found for the flagged image set(s).
            continue;

        else {
            int obj_i = all_pairs[index].getObj() / 100;
            int obj_j = all_pairs[index].getObj() % 100;
            int scene_i = all_pairs[index].getScene() / 100;
            int scene_j = all_pairs[index].getScene() % 100;

            cout << "Found relation between " << image_names[scene_i] << " and " << image_names[obj_i] << endl;

            Mat R =
                rectCamerasSet[obj_i][obj_j].getK().inv() *
                all_pairs[index].getH().inv() *
                rectCamerasSet[scene_i][scene_j].getK() *
                rectCamerasSet[scene_i][scene_j].getR();

            //relative rotation matrix is found.
            Mat R_rel = rectCamerasSet[obj_i][obj_j].getR().inv() * R;

            // all rotations in obj set are updated by reletive rotation matrix.
            for (int i = 0; i < rectCamerasSet[obj_i].size(); i++) {
                rectCamerasSet[obj_i][i].setR(rectCamerasSet[obj_i][i].getR() * R_rel);
            }
            // obj set could find a pair, so flag it as "already chosen".
            flags[obj_i] = true;
        }
    }

    // remove the image sets which do not have any relationship with any other image sets.
    for (int i = 0; i < flags.size(); ) {
        if (!flags[i]) { //no relationship
            rectImagesSet.erase(rectImagesSet.begin() + i);
            rectCamerasSet.erase(rectCamerasSet.begin() + i);
            flags.erase(flags.begin() + i);
        }
        else i++;
    }

}

/*
    The following 2 functions are used as steps of cylindrical panorama.
*/

void CustomRelationFinder::removeUnpairedImages(vector<Mat>& images, vector<PairwiseMatches>& pairs) {
    /*
        If there exist an image with no relation (overlapping) or weaker relation (bad homography), 
        we delete the image and its corresponding camera parameters from the lists (cameras, images, pairs).  
    */
    for (int i = 0; i < images.size();) {
        bool found = false;
        for (int j = 0; j < pairs.size(); j++) {
            if (i == pairs[j].getObj() || i == pairs[j].getScene()) {
                found = true;
                break;
            }
        }
        if (!found) {
            images.erase(images.begin() + i);
            for (int j = 0; j < pairs.size(); j++) {
                if (pairs[j].getObj() > i)
                    pairs[j].setObj(pairs[j].getObj() - 1);
                if (pairs[j].getScene() > i)
                    pairs[j].setScene(pairs[j].getScene() - 1);
            }
        }
        else i++;
    }
}


void CustomRelationFinder::findRelationsAmongImages(vector<Mat>& images, vector<PairwiseMatches>& pairs) {
    
    ComputeFeatures computeFeatures; // extracts features of an image
    vector<PairwiseMatches> all_pairs; // keeps track of all pairs of images.
    
    /*
        This nested loop searches for all combinations of the pairs of images that are overlapping.
    */
    for (int i = 0; i < images.size(); i++) {
        for (int j = i + 1; j < images.size(); j++) {
            
            // To differ src and destination points we choose this method: "scene  = H * obj" OR "obj = H^-1 * scene
            vector<Point2d> obj, scene;

            /*
                Extracts features of both i-th and j-th images and store those data in customly declared "PairwiseMatches" object.
            */
            int numberOfGoodMatches = computeFeatures.imageFeatureComputer(images[j], images[i], obj, scene);
            PairwiseMatches pm(j, i, obj, scene, numberOfGoodMatches);

            // We assume that there should be at least 8 good matches between i-th and j-th images to continue.
            if (obj.size() >= 8) {
                
                // Computes the homography matrix between i-th and j-th images.
                pm.computeH();
                
                /*
                    If the homography between i-th and j-th images exists and
                    If this homography is really good enough, then
                    we can add the relation between those images to all_pairs vector.
                */
                if (pm.isHomographyFound() && pm.niceHomography()) {
                    PairwiseMatches pm_inv(i, j, obj, scene, numberOfGoodMatches);
                    pm_inv.setH(pm.getH().inv());
                    all_pairs.push_back(pm);
                    all_pairs.push_back(pm_inv);
                }
            }
        }
    }

    /*
        Below what we are doing are :
        * Choose the best and final pairs among the images.
        * Estimate the focal lengths for the camera calibration matrix.
        * Estimate the rotation matrices
        * Set intrinsic and extrinsic camera parameters.
        
    */
    vector<bool> flags(images.size());
    flags[0] = true;
    for (int k = 1; k < flags.size(); k++) {
        int index = -1;
        int max = INT_MIN;
        for (int i = 0; i < flags.size(); i++) {
            if (!flags[i]) {
                for (int j = 0; j < flags.size(); j++) {
                    if (flags[j]) {
                        for (int p = 0; p < all_pairs.size(); p++) {
                            //searching best pair for flagged image(s).
                            if (all_pairs[p].getObj() == i && all_pairs[p].getScene() == j && all_pairs[p].getNumberOfInliers() > max) {
                                max = all_pairs[p].getNumberOfInliers();
                                index = p;
                            }
                        }
                    }
                }
            }
        }
        if (index == -1) // No/not good relations found for the flagged image(s).
            continue;
        else {  

            // obj and scene are taking the index of two images having best relations based on good homography.
            int obj = all_pairs[index].getObj();
            int scene = all_pairs[index].getScene();

            /* 
               When we choose an image to the pairs network, we set its flag to true
               to avoid re-checking it again.
            */
            flags[obj] = true;
            
            /*
                Choose pairs which creates stitching network for panorama.
            */
            pairs.push_back(all_pairs[index]);
        }
    }
    /*
        Remove the images and its corresponding camera parameters, 
        if those images have "no" or "not good" overlap with any other images in the list.
    */
    removeUnpairedImages(images, pairs);
}
