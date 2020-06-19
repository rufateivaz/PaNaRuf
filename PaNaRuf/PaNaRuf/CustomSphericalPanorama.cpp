#include "CustomSphericalPanorama.h"
#include "CustomRelationFinder.h"

int  CustomSphericalPanorama::applyCustomSphericalWarping(vector<String> image_names, double hfov, double vfov) {
    
    this->hfov = hfov; // horizontal field of view
    this->vfov = vfov; // vertical field of view

    /*
        For each fisheye image, we derive multiple rectilinear images.
        We store those rectilinear images in rectImageSet.
        We store the corresponding camera parameters in rectCameraSet.

        each set represents a set of rectilienar images obtained from a single fisheye image.
        Therefore, we declare set of sets [vector<vector<>>]
    */
    vector<vector<Mat>> rectImagesSet;
    vector<vector<CameraParameters>> rectCamerasSet;

    /*
        Reading each fisheye image
        Deriving multiple rectilinear images and storing in rectImagesSet.
        Deriving multiple corresponding pinhole camera parameters and storing in rectCamerasSet.
    */
    input_output.StartGettingRectilinearImages(); // printer
    for (int i = 0; i < image_names.size(); i++) {
        Mat image = imread(image_names[i]);
        if (image.empty()) {
            input_output.readingError(image_names[i]);
            return -1;
        }

        vector<Mat> subImages;
        vector<CameraParameters> subCameras;

         /*
            add each rectilinear images set to subImages set
            add each pinhole camera parameter set to subCameras set
        */
        fish2persp(image, subImages, subCameras);

        /*
            add each subImages and corresponding subCameras to rectImagesSet and rectCamerasSet.
        */
        rectImagesSet.push_back(subImages);
        rectCamerasSet.push_back(subCameras);
        
        // free unnecessary matrix and clear the vectors.
        image.release();
        subImages.clear();
        subCameras.clear();
    }

    // Checking if there are enough images to apply panorama.
    if (rectImagesSet.size() == 0)
        input_output.NoEnoughImagesException();

    /*
        Finds relations among rectliniear images set each of which is 
        derived from different fisheye images.
    */
    input_output.StartFindingRelations(); //printer
    relationFinder.findRelationsAmongImageSets(rectImagesSet, rectCamerasSet, image_names);

    /*
        - warped_images : stores each spherical image
        - warped_mask : stores each spherical mask
        - corners : stores top-left points of each spherical image
        - sizes : stores the image sizes of each spherical image
    */
    vector<Mat> warped_images;
    vector<Mat> warped_masks;
    vector<Point> corners;
    vector<Size> sizes;

    input_output.StartApplyingSphericalWarping();
    Warping(rectImagesSet, rectCamerasSet, warped_images, warped_masks, corners, sizes);

    // Starts blending the warped images using multi-band blending algorithm of OPENCV.
    input_output.StartApplyingBlending();
    Mat result;
    blending.applyMultiBandBlending(warped_masks, warped_images, corners, sizes, result);

    // Writes the output image.
    input_output.prepareOutputImage(SPHERICAL, result);
    return 0;
}

/*
    converts degree to radian
*/
double CustomSphericalPanorama::toRadian(double degree) {
    return degree * CV_PI / 180;
}

/*
    converts radian to degree
*/
double CustomSphericalPanorama::toDegree(double radian) {
    return radian * 180 / CV_PI;
}


Mat CustomSphericalPanorama::Rotation(double aroundX, double aroundY, double aroundZ) {
    
    /*
        based on the parameters given to the function,
        the function constructs 3x3 rotation matrices Rx,Ry and Rz that are rotating the
        virtual pinhole camera around x,y and z respectively.
        Then it returns  Rx * Ry * Rz.
    */
    
    Mat Rx = Mat::eye(Size(3, 3), CV_64F);
    Mat Ry = Mat::eye(Size(3, 3), CV_64F);
    Mat Rz = Mat::eye(Size(3, 3), CV_64F);

    double radX = toRadian(aroundX);
    double radY = toRadian(aroundY);
    double radZ = toRadian(aroundZ);

    Rx.at<double>(1, 1) = cos(radX);
    Rx.at<double>(1, 2) = -sin(radX);
    Rx.at<double>(2, 1) = sin(radX);
    Rx.at<double>(2, 2) = cos(radX);

    Ry.at<double>(0, 0) = cos(radY);
    Ry.at<double>(0, 2) = sin(radY);
    Ry.at<double>(2, 0) = -sin(radY);
    Ry.at<double>(2, 2) = cos(radY);

    Rz.at<double>(0, 0) = cos(radZ);
    Rz.at<double>(0, 1) = -sin(radZ);
    Rz.at<double>(1, 0) = sin(radZ);
    Rz.at<double>(1, 1) = cos(radZ);

    return Rx * Ry * Rz;
}

void CustomSphericalPanorama::findCameraParameters(vector<CameraParameters>& cameraParams, vector<Mat>& perspImages, int& numberOfImages, int wd, int hd) {

    /*
        Below, the function constructs the corresponding camera intrinsic and
        extrinsic parameters for each rectilinear image.
    */
    double hdeg = 45;
    double vdeg = 45;
    if (hfov > 220)
        hdeg = 45;
    else
        hdeg =  hfov / 4;

    if (vfov > 220)
        vdeg = 45;
    else
        vdeg = vfov / 4;

    //Camera intrinsic parameter of each taken rectilinear image
    Mat K = Mat::eye(Size(3, 3), CV_64F);
    K.at<double>(0, 0) = 0.5 * double(wd) / tan(toRadian(hdeg));
    K.at<double>(1, 1) = 0.5 * double(hd) / tan(toRadian(vdeg));
    K.at<double>(0, 2) = 0.5 * double(wd);
    K.at<double>(1, 2) = 0.5 * double(hd);

    /*
        Taking rectilinear images by rotating virtual camera vertically (around x axis),
        horizontally (around y axis) and both horizontally and vertically (around x and y axis).
    */
    double i = 0;
    while ((vfov/2-(vdeg+i)) >= 0.1) {
        if (vfov / 2 - (vdeg+i) >= vdeg)
            i += vdeg;
        else 
            i = i + (vfov / 2 - (i+vdeg));
        perspImages.push_back(Mat(Size(wd, hd), CV_8UC3));
        perspImages.push_back(Mat(Size(wd, hd), CV_8UC3));
        // Setting camera intrinsic/extrinsic parameters.
        CameraParameters camera;
        camera.setK(K);
        //rotation matrix around x axis (vertically)
        camera.setR(Rotation(i, 0, 0));  cameraParams.push_back(camera);
        camera.setR(Rotation(-i, 0, 0));  cameraParams.push_back(camera);
    }

    
    double j = 0;
    while ((hfov / 2 - (hdeg + j)) >= 0.1) {
        if (hfov / 2 - (hdeg + j) >= hdeg)
            j += hdeg;
        else
            j = j + (hfov / 2 - (j + hdeg));
        for(int i=0; i < 6 ;i++)
            perspImages.push_back(Mat(Size(wd, hd), CV_8UC3));
        // Setting camera intrinsic/extrinsic parameters.
        CameraParameters camera;
        camera.setK(K);
        //rotation matrix around y axis (horizontally)
        camera.setR(Rotation(0, j, 0));  cameraParams.push_back(camera);
        camera.setR(Rotation(0, -j, 0));  cameraParams.push_back(camera);
        //rotation matrix around y axis and x axis (horizontally and vertically)
        camera.setR(Rotation(0, j, 45));  cameraParams.push_back(camera);
        camera.setR(Rotation(0, j, -45));  cameraParams.push_back(camera);
        camera.setR(Rotation(0, -j, 45));  cameraParams.push_back(camera);
        camera.setR(Rotation(0, -j, -45));  cameraParams.push_back(camera);
    }
    // takes the number of rectilinear images taken by virtual pinhole camera.
    numberOfImages = (int)perspImages.size();
}


void CustomSphericalPanorama::fish2persp(Mat& image, vector<Mat>& images, vector<CameraParameters>& cameraParams) {
    

    int ws = image.cols; // width of fisheye image.
    int hs = image.rows; // height of fisheye image.
    double N = min(ws, hs); // minimum dimension

    int wd = 300; // by default the width of each rectilinear image is to 300
    int hd = 300; // by default the height of each rectilinear image is to 300

    int numberOfImages = 0; // the number of rectilinear image is initialized

    double F = double(N) * 0.5 / toRadian(hfov/2); //focal length of fisheye lens
    
    //the function gets camera parameters of each rectilinear image.
    findCameraParameters(cameraParams, images, numberOfImages, wd, hd);
    
    // Mapping from fisheye image points to each reactilinear image points to color each rectilinear image.
    for (int k = 0; k < numberOfImages; k++) {
        Mat Rinv_Kinv = cameraParams[k].getR().inv() * cameraParams[k].getK().inv();
#pragma omp parallel for
        for (int i = 0; i < wd; i++) {
            for (int j = 0; j < hd; j++) {
                // 2D rectilinear image
                Mat p2d = Mat(Size(1, 3), CV_64F);
                p2d.at<double>(0, 0) = double(i);
                p2d.at<double>(1, 0) = double(j);
                p2d.at<double>(2, 0) = 1.0;

                // to 3D
                Mat p3d = Rinv_Kinv * p2d;
                double r = sqrt(pow(p3d.at<double>(0, 0), 2) + pow(p3d.at<double>(1, 0), 2) + pow(p3d.at<double>(2, 0), 2));
                
                //to scene 
                double theta = acos(p3d.at<double>(2, 0) / r);
                double phi = atan2(p3d.at<double>(1, 0), p3d.at<double>(0, 0));
                
                // to 2D fisheye image 
                double ru = F * theta;
                double x_fish = 0.5 * double(ws) + ru * cos(phi);
                double y_fish = 0.5 * double(hs) + ru * sin(phi);


                /*
                    Bilinear interpolation method is used to color each rectilinear image point 
                    with the corresponding pixel value of (x_fish, y_fish) in fisheye iamge.
                */
                if (x_fish >= 1.0 && x_fish <= (ws - 1.0) &&
                    y_fish >= 1.0 && y_fish <= (hs - 1.0)) {
                    
                    if (floor(x_fish) != ceil(x_fish) &&
                        floor(y_fish) != ceil(y_fish)) {
                        images[k].at<Vec3b>(j, i) = utils.BilinearInterpolation(x_fish, y_fish, image);
                    }
                    else {
                        images[k].at<Vec3b>(j, i) = image.at<Vec3b>((int)round(y_fish), (int)round(x_fish));
                    }
                }
                else {
                    if (round(x_fish) >= 0.0 && round(x_fish) < 1.0 &&
                        round(y_fish) >= 0.0 && round(y_fish) < 1.0) {
                        images[k].at<Vec3b>(j, i) = image.at<Vec3b>((int)round(y_fish), (int)round(x_fish));
                    }
                }
            }
        }
    }
}


void CustomSphericalPanorama::Warping(vector<vector<Mat>> images, vector<vector<CameraParameters>> cameraParams, vector<Mat>& warped_images, vector<Mat>& warped_masks, vector<Point>& corners, vector<Size>& sizes) {
   /*
        Counting the number of rectilinear images in all.
   */
    int count = 0;
    int counter = 0;
    for (int i = 0; i < images.size(); i++) {
        for (int j = 0; j < images[i].size(); j++) {
            count++;
        }
    }

    //Loop over each rectilinear image and apply the following.
    for (int i = 0; i < images.size(); i++) {
        for (int j = 0; j < images[i].size(); j++) {

            cout << "Warping " << ++counter << "/" << count << endl;

            /*
                Creating border-reflect effect around the rectilinear image
                which we need for multi-band blending.
            */
            Mat border_reflected_image;
            utils.addReflectiveImagesAround(images[i][j], border_reflected_image);

            // Applying forward warping for ij-th rectilinear image.
            Rect roi = forwardWarping(images[i][j], cameraParams[i][j].getK(), cameraParams[i][j].getR()); //applies forward warping and finds size of the dst/warped image.
            if (!roi.empty() && roi.area() == 0)
                continue;
            else
                if (roi.empty())
                    continue;

            /*
                After finding the top-left corner point of each spherical image along with its size,
                we keep track of each of those data which we need in applying multi-band blending.
            */
            corners.push_back(roi.tl()); // top-left corner points of each spherical image.
            sizes.push_back(roi.size()); // size of each spherical image.

            Mat warped_image, warped_mask; // destination (spherical) image and destination mask
            
            /*
                Apply backward mapping for ij-th spherical image.
            */
            BackwardWarping(border_reflected_image, roi, warped_image, warped_mask, cameraParams[i][j].getK(), cameraParams[i][j].getR());
            
            // add each warped spherical images and corresponding masks to the vectors below.
            warped_images.push_back(warped_image);
            warped_masks.push_back(warped_mask);

            // free all of the created matrices.
            warped_image.release();
            warped_mask.release();
            border_reflected_image.release();
        }
    }
}

Rect CustomSphericalPanorama::forwardWarping(Mat image, Mat K, Mat R) {
    int w = image.cols; // width of the rectilinear image
    int h = image.rows; // height of the rectilinear image
    Point2i topLeft = Point2i(INT_MAX, INT_MAX); // top-left point of the spherical image
    Point2i bottomRight = Point2i(INT_MIN, INT_MIN); // bottom-right of the spherical image

    /*
        Instead of multiplying R^-1 by K^-1 at each iteration, we multiply it beforehand
        to speed-up the program.
    */
    Mat Rinv_Kinv = R.inv() * K.inv();

    /*
        Walk over each rectilinear image points and apply the forward mapping to get
        the destination (spherical) image points.
    */
    
    //definition of lock which handles critical section problem for multi-threading
    omp_lock_t writelock;
    omp_init_lock(&writelock);
#pragma omp parallel for 
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {

            //2D Rectilinear Image points.
            Mat p2d = Mat(Size(1, 3), CV_64F);
            p2d.at<double>(0, 0) = double(i);
            p2d.at<double>(1, 0) = double(j);
            p2d.at<double>(2, 0) = 1.0;

            //2D-3D un-projection and rotation.
            Mat point3d = Rinv_Kinv * p2d;

            //scale value is sqrt (X^2 + Y^2 + Z^2)
            double scale_to_sph = sqrt(pow(point3d.at<double>(0, 0), 2) + pow(point3d.at<double>(1, 0), 2) + pow(point3d.at<double>(2, 0), 2));

            //3D to unit-sphere coordinates.
            Mat point3d_sph = point3d / scale_to_sph;

            //Longitude and Latitude (OR theta and phi) values 
            Mat sph = Mat(Size(1, 3), CV_64F);
            sph.at<double>(0, 0) = atan2(point3d_sph.at<double>(0, 0), point3d_sph.at<double>(2, 0));
            sph.at<double>(1, 0) = atan2(point3d_sph.at<double>(1, 0), sqrt(pow(point3d_sph.at<double>(0, 0), 2) + pow(point3d_sph.at<double>(2, 0), 2)));
            sph.at<double>(2, 0) = 1.0;

            //to 2D spherical image points.
            double x_dst = K.at<double>(0, 0) * sph.at<double>(0, 0);
            double y_dst = K.at<double>(1, 1) * sph.at<double>(1, 0);

            /*
                The following operations are applied to get minimum and maximum of spherical image points,
                where min is top-left and max is bottom-right.
            */
            //only one thread can process the following at a time.
            omp_set_lock(&writelock);
                if (topLeft.x > (int)round(x_dst))
                    topLeft.x = (int)round(x_dst);
                if (topLeft.y > (int)round(y_dst))
                    topLeft.y = (int)round(y_dst);
                if (bottomRight.x < (int)round(x_dst))
                    bottomRight.x = (int)round(x_dst);
                if (bottomRight.y < (int)round(y_dst))
                    bottomRight.y = (int)round(y_dst);
            omp_unset_lock(&writelock);
        }
    }

    if (topLeft.x == INT_MAX || topLeft.y == INT_MAX || bottomRight.x == INT_MIN || bottomRight.y == INT_MIN)
        return Rect(Point(0, 0), Point(0, 0));
    return Rect(topLeft, bottomRight);
}


void CustomSphericalPanorama::BackwardWarping(Mat image, Rect roi, Mat& warped_image, Mat& warped_mask, Mat K, Mat R) {
    int ws = image.cols; // width of rectilinear image.
    int hs = image.rows; // height of rectilinear image.
    int wd = roi.br().x - roi.tl().x; // width of destination (spherical) image.
    int hd = roi.br().y - roi.tl().y; // height of destination (spherical) image.

    cout << wd << "x" << hd << endl;

    // offset is used to shift the spherical image point to actual point in x-y image coordinates. 
    Point2d offset = 0.5 * Point2d((roi.tl() + roi.br())) - Point2d(double(wd) * 0.5, double(hd) * 0.5);

    // warped_image is destination (spherical) image
    warped_image = Mat(Size(wd, hd), CV_8UC3, Scalar(0));

    // warped_mask is destination (spherical) mask
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

            // un-unproject from 2D spherical image coordinate to get theta and phi respectively.
            Mat sph = Mat(Size(1, 3), CV_64F);
            sph.at<double>(0, 0) = (x + offset.x) / K.at<double>(0, 0);
            sph.at<double>(1, 0) = (y + offset.y) / K.at<double>(1, 1);
            sph.at<double>(2, 0) = 1.0;

            // Obtain unit-sphere coordinates from theta and phi
            Mat point3d = Mat(Size(1, 3), CV_64F);
            point3d.at<double>(0, 0) = sin(sph.at<double>(0, 0)) * cos(sph.at<double>(1, 0));
            point3d.at<double>(1, 0) = sin(sph.at<double>(1, 0));
            point3d.at<double>(2, 0) = cos(sph.at<double>(0, 0)) * cos(sph.at<double>(1, 0));

            // pass to 3D coordinates. We use fabs since z-value can also be negative.
            point3d = point3d / fabs(point3d.at<double>(2, 0));

            // Rotate in 3D and project to 2D rectilinear image coordinate.
            Mat point2d = KR * point3d;

            // If the point is beyond camera, do not take it.
            if (point2d.at<double>(2, 0) <= 0)
                continue;

            // normalize to [x,y,1]
            point2d = point2d / point2d.at<double>(2, 0);

            // corresponding point
            Point2d pixel = Point2d(point2d.at<double>(0, 0), point2d.at<double>(1, 0));

            /*
                Since we added self-reflected images around the rectilinear image. We
                need to shift the point from top left to middle.
            */
            pixel.x += ((double)ws / 2.0 - (double)ws / 6.0);
            pixel.y += ((double)hs / 2.0 - (double)hs / 6.0);

            /* Below, we check if the requested pixel value is within the rectilinear image borders,
                * If yes, we apply bilinear-interpolation method to color each pixel
                  of destination (spherical) image. And, we give white color to mask.
                * If no,  we ignore the current pixel value of destination (spherical) image and
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



