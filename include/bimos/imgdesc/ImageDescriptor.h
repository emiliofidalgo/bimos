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

#ifndef IMAGEDESC_H
#define IMAGEDESC_H

#include <bimos/imgdesc/ldb.h>
#include <bimos/imgdesc/ORBextractor.h>
#include <opencv2/xfeatures2d.hpp>

namespace bimos
{

/**
 * @brief Class for describing an image using local features.
 */
class ImageDescriptor
{
public:
    ImageDescriptor(const std::string& method = "FAST_LDB", const int nfeatures = 2500);
    ~ImageDescriptor();

    void describeImage(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

private:

    // Pointer to the correspondent function
    void (ImageDescriptor::*functPtr)(const cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&);

    // Implementations for the different description methods
    void describeImage_FASTBRIEF(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);
    void describeImage_FASTLDB(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);
    void describeImage_ORBBRIEF(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);
    void describeImage_ORBORB(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);
    void describeImage_ORBLDB(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descs);

    // Description parameters
    std::string _method;
    int _nfeatures;

    // Objects for keypoint detection and description
    cv::Ptr<cv::FastFeatureDetector> _fastdet;
    ORB_SLAM::ORBextractor _orb;
    cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> _briefdes;
    LDB _ldbdes;
};

}
#endif // IMAGEDESC_H
