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

#include <bimos/imgdesc/ImageDescriptor.h>

namespace bimos
{

/**
 * @brief Default constructor.
 * @param method Combination of detector/descriptor to be used during the description process.
 * @param nfeatures Total number of keypoints to obtain from the image.
 */
ImageDescriptor::ImageDescriptor(const std::string& method, const int nfeatures) :
    _method(method),
    _nfeatures(nfeatures),
    _fastdet(new cv::FastAdjuster(10), nfeatures, nfeatures + 250, 300),
    _orb(nfeatures, 1.2, 8, ORB_SLAM::ORBextractor::FAST_SCORE, 10),
    _briefdes(32),
    _ldbdes(48)
{
    if (method == "FAST_BRIEF")
    {
        functPtr = &ImageDescriptor::describeImage_FASTBRIEF;
    }
    else if (method == "FAST_LDB")
    {
        functPtr = &ImageDescriptor::describeImage_FASTLDB;
    }
    else if (method == "ORB_BRIEF")
    {
        functPtr = &ImageDescriptor::describeImage_ORBBRIEF;
    }
    else if (method == "ORB_ORB")
    {
        functPtr = &ImageDescriptor::describeImage_ORBORB;
    }
    else if (method == "ORB_LDB")
    {
        functPtr = &ImageDescriptor::describeImage_ORBLDB;
    }
}

/**
 * @brief Default Destructor.
 */
ImageDescriptor::~ImageDescriptor()
{
}

/**
 * @brief Public method used as entry point for image description.
 * @param image The image to be described.
 * @param kps Detected keypoints.
 * @param descs Descriptors of the keypoints.
 */
void ImageDescriptor::describeImage(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs)
{
    (this->*functPtr)(image, kps, descs);
}

/**
 * Describes the image using the FAST/BRIEF method.
 * @brief ImageDescriptor::describeImage_FASTBRIEF
 * @param image \see ImageDescriptor::describeImage
 * @param kps \see ImageDescriptor::describeImage
 * @param descs \see ImageDescriptor::describeImage
 */
void ImageDescriptor::describeImage_FASTBRIEF(const cv::Mat &image, std::vector<cv::KeyPoint> &kps, cv::Mat &descs)
{    
    _fastdet.detect(image, kps);
    _briefdes.compute(image, kps, descs);
}

/**
 * Describes the image using the FAST/LDB method.
 * @brief ImageDescriptor::describeImage_FASTBRIEF
 * @param image \see ImageDescriptor::describeImage
 * @param kps \see ImageDescriptor::describeImage
 * @param descs \see ImageDescriptor::describeImage
 */
void ImageDescriptor::describeImage_FASTLDB(const cv::Mat &image, std::vector<cv::KeyPoint> &kps, cv::Mat &descs)
{    
    _fastdet.detect(image, kps);
    _ldbdes.compute(image, kps, descs, true);
}

/**
 * Describes the image using the ORB/BRIEF method.
 * @brief ImageDescriptor::describeImage_FASTBRIEF
 * @param image \see ImageDescriptor::describeImage
 * @param kps \see ImageDescriptor::describeImage
 * @param descs \see ImageDescriptor::describeImage
 */
void ImageDescriptor::describeImage_ORBBRIEF(const cv::Mat &image, std::vector<cv::KeyPoint> &kps, cv::Mat &descs)
{    
    _orb(image, cv::Mat(), kps, descs);
    descs.release();
    _briefdes.compute(image, kps, descs);
}

/**
 * Describes the image using the ORB/ORB method.
 * @brief ImageDescriptor::describeImage_FASTBRIEF
 * @param image \see ImageDescriptor::describeImage
 * @param kps \see ImageDescriptor::describeImage
 * @param descs \see ImageDescriptor::describeImage
 */
void ImageDescriptor::describeImage_ORBORB(const cv::Mat &image, std::vector<cv::KeyPoint> &kps, cv::Mat &descs)
{    
    _orb(image, cv::Mat(), kps, descs);
}

/**
 * Describes the image using the ORB/LDB method.
 * @brief ImageDescriptor::describeImage_FASTBRIEF
 * @param image \see ImageDescriptor::describeImage
 * @param kps \see ImageDescriptor::describeImage
 * @param descs \see ImageDescriptor::describeImage
 */
void ImageDescriptor::describeImage_ORBLDB(const cv::Mat &image, std::vector<cv::KeyPoint> &kps, cv::Mat &descs)
{    
    _orb(image, cv::Mat(), kps, descs);
    descs.release();
    _ldbdes.compute(image, kps, descs, false);
}

}
