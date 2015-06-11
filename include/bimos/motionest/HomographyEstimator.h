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

#ifndef HOMOGRAPHYESTIMATOR_H
#define HOMOGRAPHYESTIMATOR_H

#include <limits>

#include <bimos/util/Image.h>
#include <bimos/motionest/RANSAC.h>
#include <bimos/motionest/AffineEstimator2D.h>
#include <bimos/motionest/SimilarityEstimator2D.h>
#include <bimos/util/Transform.h>
#include <bimos/util/util.h>

namespace bimos
{

struct HomographyEstimator
{
    static bool estimate(Image* image_prev, Image* image, cv::Mat_<double>& H, std::vector<cv::DMatch>& matches_inliers, double& rep_error, double ratio = 0.8, double hom_delta = 7.0, TransformType type = SIMILARITY);
};

}

#endif // HOMOGRAPHYESTIMATOR_H
