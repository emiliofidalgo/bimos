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

#include "bimos/motionest/HomographyEstimator.h"

namespace bimos
{

double inline SQF(double x) { return x * x; }

bool HomographyEstimator::estimate(Image* image_prev, Image* image, cv::Mat_<double>& H, std::vector<cv::DMatch>& matches_inliers, double& rep_error, double hom_delta, TransformType type)
{
    // Matching images
    std::vector<cv::DMatch> matches;
    ratioMatching(image, image_prev, matches);

    rep_error = std::numeric_limits<double>::infinity();
    bool correct = false;

    // Processing the resulting matchings.
    std::vector<PointPair2D> pointData;
    std::vector<PointPair2D*> pointDataPtr;
    for(size_t match_ind = 0; match_ind < matches.size(); match_ind++)
    {
        PointPair2D* pp = new PointPair2D;

        // Query point
        cv::Point2f p = image->kps[matches[match_ind].queryIdx].pt;
        pp->second.x = p.x;
        pp->second.y = p.y;

        // Train point
        p = image_prev->kps[matches[match_ind].trainIdx].pt;
        pp->first.x = p.x;
        pp->first.y = p.y;

        pointDataPtr.push_back(pp);
        pointData.push_back(*pp);
    }

    // Classes for estimate the homography
    std::vector<double> parameters;
    std::vector<int> outliers;

    ParameterEstimator<PointPair2D,double>* estimator;
    if (type == SIMILARITY)
    {
        estimator = new SimilarityEstimator2D(SQF(hom_delta));
    }
    else
    {
        estimator = new AffineEstimator2D(SQF(hom_delta));
    }

    // Performing RANSAC
    double error;
    RANSAC<PointPair2D, double>::compute(parameters, estimator, pointData, 0.99, true, outliers, &error);
    rep_error = error;

    // Filtering matches according to the outliers
    for (int i = 0; i < outliers.size(); i++)
    {
        int match_ind = outliers[i];
        matches[match_ind].queryIdx = -1;
    }

    // Deleting matches
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].queryIdx != -1)
        {
            matches_inliers.push_back(matches[i]);
        }
    }

    cv::Mat_<double> Hs = cv::Mat::eye(3, 3, CV_64F);
    // if the process finished successful
    if (parameters.size() > 0)
    {
        correct = true;
        if (type == SIMILARITY)
        {
            double sc = parameters[0];
            double ss = parameters[1];
            double tx = parameters[2];
            double ty = parameters[3];

            Hs(0, 0) =  sc; Hs(0, 1) = -ss; Hs(0, 2) = tx;
            Hs(1, 0) =  ss; Hs(1, 1) =  sc; Hs(1, 2) = ty;
        }
        else if (type == AFFINE)
        {
            double a = parameters[0];
            double b = parameters[1];
            double c = parameters[2];
            double d = parameters[3];
            double e = parameters[4];
            double f = parameters[5];

            Hs(0, 0) =  a; Hs(0, 1) = b; Hs(0, 2) = e;
            Hs(1, 0) =  c; Hs(1, 1) = d; Hs(1, 2) = f;
        }
    }

    // Deleting memory
    pointData.clear();
    for(unsigned i = 0; i < pointDataPtr.size(); i++)
    {
        delete pointDataPtr[i];
    }
    delete estimator;

    Hs.copyTo(H);

    return correct;
}

}
