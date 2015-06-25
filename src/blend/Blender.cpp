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

#include <bimos/blend/Blender.h>

namespace bimos
{

/**
 * @brief Default constructor.
 */
Blender::Blender()
{    
}

/**
 * @brief Default destructor.
 */
Blender::~Blender()
{    
}

/**
 * @brief Entry point for running this class as a thread.
 */
void Blender::run()
{
    double init_time = omp_get_wtime();

    // Getting the current state of the graph
    std::vector<Transform> transforms;
    mgraph->getKFTransforms(transforms);

    int nimages = transforms.size();

    std::vector<cv::Mat> masks(nimages);
    std::vector<cv::Point> corners(nimages);
    std::vector<cv::Size> sizes(nimages);
    std::vector<cv::Mat> images_warped(nimages);
    std::vector<cv::Mat> images_warped_f(nimages);
    std::vector<cv::Mat> masks_warped(nimages);

    // Warping images
    ROS_INFO("[blender] Warping images ...");
    for (int i = 0; i < transforms.size(); i++)
    {
        Keyframe* kf = mgraph->getKeyframe(i);

        // Prepare image mask.
        masks[i].create(kf->image->image.size(), CV_8U);
        masks[i].setTo(cv::Scalar::all(255));

        // Warping image
        corners[i] = warpImg(kf->image->image, transforms[i], images_warped[i], cv::INTER_LINEAR, cv::BORDER_REFLECT);
        sizes[i] = images_warped[i].size();

        // Warping mask
        warpImg(masks[i], transforms[i], masks_warped[i], cv::INTER_NEAREST, cv::BORDER_CONSTANT);

        // Passing the warped image to float.
        images_warped[i].convertTo(images_warped_f[i], CV_32F);
    }

    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    if (p->blend_exp)
    {
        ROS_INFO("[blender] Compensating exposures ...");
        compensator = cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN_BLOCKS);
        compensator->feed(corners, images_warped, masks_warped);
    }

    if (p->blend_seams)
    {
        ROS_INFO("[blender] Finding seams ...");
        cv::Ptr<cv::detail::SeamFinder> seam_finder = new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
        seam_finder->find(images_warped_f, corners, masks_warped);
        images_warped_f.clear();
    }

    ROS_INFO("[blender] Blending the final mosaic ...");
    cv::Ptr<cv::detail::Blender> blender = cv::detail::Blender::createDefault(cv::detail::Blender::MULTI_BAND, false);
    cv::Size dst_sz = resultRoi(corners, sizes).size();
    float blend_width = std::sqrt(static_cast<float>(dst_sz.area())) * 5.0 / 100.f;
    cv::detail::MultiBandBlender* mb = dynamic_cast<cv::detail::MultiBandBlender*>(static_cast<cv::detail::Blender*>(blender));
    mb->setNumBands(static_cast<int>(std::ceil(std::log(blend_width) / std::log(2.)) - 1.));
    // Prepare blender
    blender->prepare(corners, sizes);
    cv::Mat image_warped_s;
    cv::Mat dilated_mask, seam_mask;
    for (int i = 0; i < nimages; i++)
    {
        ROS_INFO("[blender] Compositing image %i ...", i);

        if (p->blend_exp)
        {
            // Compensate exposure
            compensator->apply(i, corners[i], images_warped[i], masks_warped[i]);
        }

        images_warped[i].convertTo(image_warped_s, CV_16S);

        cv::dilate(masks_warped[i], dilated_mask, cv::Mat());
        cv::resize(dilated_mask, seam_mask, masks_warped[i].size());
        cv::Mat mask_warped = seam_mask & masks_warped[i];

        blender->feed(image_warped_s, mask_warped, corners[i]);
    }

    // Blending the image
    cv::Mat result, result_mask;
    blender->blend(result, result_mask);

    double end_time = omp_get_wtime();

    //  Saving the results
    std::string pano_filename = p->working_dir + "mosaic_" + SSTR(mosaic_id) + ".jpg";
    ROS_INFO("[blender] Saving mosaic to %s", pano_filename.c_str());
    cv::imwrite(pano_filename, result);

    std::string pano_filename2 = p->working_dir + "mosaic_" + SSTR(mosaic_id) + "_r2.jpg";
    cv::Mat pano_r2;
    cv::resize(result, pano_r2, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
    cv::imwrite(pano_filename2, pano_r2);

    std::string pano_filename4 = p->working_dir + "mosaic_" + SSTR(mosaic_id) + "_r4.jpg";
    cv::Mat pano_r4;
    cv::resize(pano_r2, pano_r4, cv::Size(), 0.5, 0.5, CV_INTER_AREA);
    cv::imwrite(pano_filename4, pano_r4);    

    mosaic_id++;

    ROS_INFO("[blend] Blending time: %f", end_time - init_time);
}

/**
 * @brief Set params variable.
 * @param _p Pointer to params.
 */
void Blender::setParams(Params *_p)
{
    p = _p;
}

/**
 * @brief Set graph.
 * @param _mgraph Pointer to MosaicGraph.
 */
void Blender::setGraph(MosaicGraph *_mgraph)
{
    mgraph = _mgraph;
}

void Blender::detectResultRoi(cv::Size src_size, Transform& t, cv::Point& dst_tl, cv::Point& dst_br)
{
    float tl_uf = std::numeric_limits<float>::max();
    float tl_vf = std::numeric_limits<float>::max();
    float br_uf = -std::numeric_limits<float>::max();
    float br_vf = -std::numeric_limits<float>::max();
    float u, v;

    std::vector<cv::Point2f> img_points(4);
    std::vector<cv::Point2f> img_points_tr(4);
    img_points[0] = cv::Point(0, 0);
    img_points[1] = cv::Point(0, static_cast<float>(src_size.height - 1));
    img_points[2] = cv::Point(static_cast<float>(src_size.width - 1), 0);
    img_points[3] = cv::Point(static_cast<float>(src_size.width - 1), static_cast<float>(src_size.height - 1));
    cv::perspectiveTransform(img_points, img_points_tr, t.H);

    u = img_points_tr[0].x; v = img_points_tr[0].y;
    tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
    br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);

    u = img_points_tr[1].x; v = img_points_tr[1].y;
    tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
    br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);

    u = img_points_tr[2].x; v = img_points_tr[2].y;
    tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
    br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);

    u = img_points_tr[3].x; v = img_points_tr[3].y;
    tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
    br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);

    dst_tl.x = static_cast<int>(tl_uf);
    dst_tl.y = static_cast<int>(tl_vf);
    dst_br.x = static_cast<int>(br_uf);
    dst_br.y = static_cast<int>(br_vf);
}

cv::Rect Blender::buildMaps(const cv::Size src_size, Transform& t, cv::Mat& xmap, cv::Mat& ymap)
{
    cv::Point dst_tl, dst_br;
    detectResultRoi(src_size, t, dst_tl, dst_br);

    xmap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);
    ymap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);

    cv::Mat_<double> Hinv = t.inv().H;

    for (int v = dst_tl.y; v <= dst_br.y; ++v)
    {
        for (int u = dst_tl.x; u <= dst_br.x; ++u)
        {
            std::vector<cv::Point2f> img_points(1);
            std::vector<cv::Point2f> img_points_tr(1);
            img_points[0] = cv::Point(u, v);
            cv::perspectiveTransform(img_points, img_points_tr, Hinv);
            xmap.at<float>(v - dst_tl.y, u - dst_tl.x) = img_points_tr[0].x;
            ymap.at<float>(v - dst_tl.y, u - dst_tl.x) = img_points_tr[0].y;
        }
    }

    return cv::Rect(dst_tl, dst_br);
}

cv::Point Blender::warpImg(const cv::Mat& src, Transform& t, cv::Mat& dst, int interp_mode, int border_mode)
{
    cv::Mat xmap, ymap;
    cv::Rect dst_roi = buildMaps(src.size(), t, xmap, ymap);

    dst.create(dst_roi.height + 1, dst_roi.width + 1, src.type());
    remap(src, dst, xmap, ymap, interp_mode, border_mode);

    return dst_roi.tl();
}

cv::Rect Blender::resultRoi(const std::vector<cv::Point>& corners, const std::vector<cv::Size>& sizes)
{
    CV_Assert(sizes.size() == corners.size());
    cv::Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    cv::Point br(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        tl.x = std::min(tl.x, corners[i].x);
        tl.y = std::min(tl.y, corners[i].y);
        br.x = std::max(br.x, corners[i].x + sizes[i].width);
        br.y = std::max(br.y, corners[i].y + sizes[i].height);
    }
    return cv::Rect(tl, br);
}

}
