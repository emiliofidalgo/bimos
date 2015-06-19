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

#ifndef MBLENDER_H
#define MBLENDER_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

#include <bimos/graph/MosaicGraph.h>
#include <bimos/util/Params.h>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

namespace bimos
{

/**
 * @brief This class is executed in a thread in order to blend the current situation of the mosaic.
 */
class Blender
{
public:
    Blender();
    ~Blender();

    void run();

    void setParams(Params* _p);
    void setGraph(MosaicGraph* _mgraph);

private:
    // Parameters
    Params* p;

    // Graph Management
    MosaicGraph* mgraph;

    // Mosaic ID
    int mosaic_id;

    // Methods for supporting the blending process
    void detectResultRoi(cv::Size src_size, Transform& t, cv::Point& dst_tl, cv::Point& dst_br);
    cv::Rect buildMaps(const cv::Size src_size, Transform& t, cv::Mat& xmap, cv::Mat& ymap);
    cv::Point warpImg(const cv::Mat& src, Transform& t, cv::Mat& dst, int interp_mode, int border_mode);
    cv::Rect resultRoi(const std::vector<cv::Point>& corners, const std::vector<cv::Size>& sizes);
};

}
#endif // MBLENDER_H
