/**
* This file is part of bimos.
*
* Copyright (C) 2015 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
* For more information see: <http://dmi.uib.es/~egarcia>
*
* bimos is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* bimos is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with bimos. If not, see <http://www.gnu.org/licenses/>.
*/

#include <bimos/kfsel/KeyframeSelector.h>

namespace bimos
{

/**
 * @brief Default constructor.
 */
KeyframeSelector::KeyframeSelector(const ros::NodeHandle& nh, Params* params, MosaicGraph* _mgraph) :
    _nh(nh),
    _it(_nh),
    p(params),
    mgraph(_mgraph),
    lvkf_image(0),
    lvkf_rerror(0),
    nimages(0),
    nkfs(0)
{
    imgdesc = new ImageDescriptor(p->img_descriptor, p->nkeypoints);
}

/**
 * @brief Default destructor.
 */
KeyframeSelector::~KeyframeSelector()
{
    delete imgdesc;
}

/**
 * @brief Entry point for running this class as a thread.
 */
void KeyframeSelector::run()
{
    nimages = 0;
    nkfs = 0;

    if (p->batch)
    {
        // Loading the images directly from disk
        std::vector<std::string> img_filenames;
        getImageFilenames(p->batch_images_dir, img_filenames);

        ros::Rate r(200);
        for (unsigned i = 0; i < img_filenames.size(); i++)
        {
            cv::Mat img = cv::imread(img_filenames[i]);
            processImage(img);
            r.sleep();
        }
    }
    else
    {
        // Launching the topic for receiving images
        _img_subs = _it.subscribe("image", 5, &KeyframeSelector::receiveImage, this);
        ros::Rate r(50);
        while (ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }
}

/**
 * @brief Process each image that arrives to the mosaicing algorithm.
 * @param msg Image as a ROS message.
 */
void KeyframeSelector::receiveImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Converting the image to OpenCV
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    processImage(cv_ptr->image);
}

void KeyframeSelector::processImage(const cv::Mat& img)
{
    // Serialize the image on disk
    std::string image_filename_orig = p->working_dir + "images/image%06d.jpg";
    char name_orig[500];
    sprintf(name_orig, image_filename_orig.c_str(), nimages);
    cv::imwrite(name_orig, img);

    // Creating a instance of the bimos::Image class and filling the structure
    Image* image = new Image;
    image->id = nimages;
    image->filename = std::string(name_orig);
    img.copyTo(image->image);
    imgdesc->describeImage(image->image, image->kps, image->dscs);
    ROS_INFO("[kfsel] Found %lu keypoints in image %i", image->kps.size(), nimages);
    nimages++;
    mgraph->incrNumImages();

    // If the image is the first one, it is considered as the first keyframe
    if (image->id == 0)
    {
        mgraph->addKeyframe(image, 1.0, cv::Mat());
        ROS_INFO("[kfsel] Adding KF 0 to the graph");
    }
    else
    {
        // Homography computation
        Keyframe* last_kf = mgraph->getLastInsertedKF();
        ROS_INFO("[kfsel] Estimating homography between KF %i and image %i ...", last_kf->id, image->id);
        cv::Mat_<double> H;
        std::vector<cv::DMatch> inliers;
        double rep_error;
        HomographyEstimator::estimate(last_kf->image, image, H, inliers, rep_error, p->match_ratio, p->max_reproj_error);

        // Iterating for each match
        std::vector<cv::Point2f> tpoints;
        std::vector<cv::Point2f> qpoints;
        for (unsigned match_ind = 0; match_ind < inliers.size(); match_ind++)
        {
            int train_id = inliers[match_ind].trainIdx;
            int query_id = inliers[match_ind].queryIdx;
            cv::Point2f tpoint = last_kf->image->kps[train_id].pt;
            cv::Point2f qpoint = image->kps[query_id].pt;
            tpoints.push_back(tpoint);
            qpoints.push_back(qpoint);
        }
        cv::Rect tbox = cv::boundingRect(tpoints);
        cv::Rect qbox = cv::boundingRect(qpoints);

        double overlap_train = double(tbox.area()) / (last_kf->image->image.cols * last_kf->image->image.rows);
        double overlap_query = double(qbox.area()) / (image->image.cols * image->image.rows);
        double overlap = cv::min(overlap_train, overlap_query);

        ROS_INFO("[kfsel] Inliers %i, Overlap: %f, Mean Reprojection Error: %f", static_cast<int>(inliers.size()), overlap, rep_error);

        if (inliers.size() > p->kf_min_inliers && overlap > p->kf_overlap)
        {
            // This image is valid to be considered as a KF
            if (lvkf_image)
            {
                delete lvkf_image;
                lvkf_inliers.clear();
            }

            lvkf_image = image;
            lvkf_rerror = rep_error;
            lvkf_H = H;
            lvkf_inliers = inliers;
        }
        else
        {
            if (image->id == 1)
            {
                ROS_ERROR("[kfsel] The second received image needs to be considered as a keyframe.");
                ROS_ERROR("[kfsel] In order to achieve this, you can reduce the 'kf_min_inliers' and/or 'kf_overlap' parameters or increment the 'max_reproj_error' parameter.");
                ROS_ERROR("[kfsel] If it is not possible, perhaps the dataset does not present enough overlap between consecutive images to obtain a mosaic.");
                exit(0);
            }

            // Current image cannot be considered a KF

            // Adding the previous image as KF in the graph
            int nkf = mgraph->addKeyframe(lvkf_image, lvkf_rerror, lvkf_H);
            mgraph->incrObsOK();
            Keyframe* new_kf = mgraph->getLastInsertedKF();
            mgraph->addConstraints(last_kf, new_kf, lvkf_inliers);
            saveMatchings(last_kf->id, new_kf->id, p->working_dir + "inliers/", lvkf_inliers);
            ROS_INFO("[kfsel] Adding KF %i to the graph", nkf);

            // Recomputing the transformation from the current image to the current KF
            ROS_INFO("[kfsel] Recomputing homography between KF %i and image %i ...", new_kf->id, image->id);
            cv::Mat_<double> H_n;
            std::vector<cv::DMatch> inliers_n;
            double rep_error_n;
            HomographyEstimator::estimate(new_kf->image, image, H_n, inliers_n, rep_error_n, p->match_ratio, p->max_reproj_error);

            // Assigning this image as the new possible KF
            lvkf_image = image;
            lvkf_rerror = rep_error_n;
            lvkf_H = H_n;
            lvkf_inliers = inliers_n;
            ROS_INFO("[kfsel] Inliers %i, Mean Reprojection Error: %f", static_cast<int>(inliers_n.size()), rep_error_n);
        }
    }
}

}
