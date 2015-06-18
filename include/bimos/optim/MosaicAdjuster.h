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

#ifndef MOSAICADJUSTER_H
#define MOSAICADJUSTER_H

#include <boost/thread.hpp>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

#include <bimos/graph/Keyframe.h>

namespace bimos
{

class MosaicAdjuster
{
public:
    MosaicAdjuster(const double _rfactor = 8.0);
    ~MosaicAdjuster();

    void addConstraints(Keyframe* kf_prev, Keyframe* kf, const std::vector<cv::DMatch>& matches);
    void adjust(const ceres::Solver::Options& options, ceres::Solver::Summary& summary);
    void reset();

protected:
    ceres::Problem* _problem;

    // Regularization factor.
    double rfactor;

    // Mutex to control the access to the adjuster.
    boost::mutex mutex_adjuster;
};

class AdjusterCostFunctorSim
{
public:
    AdjusterCostFunctorSim(const cv::Point2f query, const cv::Point2f train, double reg_factor)
        : _query_x(query.x), _query_y(query.y), _train_x(train.x), _train_y(train.y), _reg_factor(reg_factor)
    {}

    template <typename T>
    bool operator()(const T* const params_prev, const T* const params, T* residuals) const
    {
        T a = params[0];
        T b = params[1];
        T c = params[2];
        T d = params[3];

        T ap = params_prev[0];
        T bp = params_prev[1];
        T cp = params_prev[2];
        T dp = params_prev[3];

        // --- Computing the error in backward direction ---
        T den = ap * ap + bp * bp;
        T ainv = ap / den;
        T binv = -bp / den;
        T cinv = -(ap * cp + bp * dp) / den;
        T dinv = -(ap * dp - bp * cp) / den;

        T x = T(_query_x);
        T y = T(_query_y);
        T xm = (a * x - b * y + c);
        T ym = (b * x + a * y + d);
        T xi = ainv * xm - binv * ym + cinv;
        T yi = binv * xm + ainv * ym + dinv;
        residuals[0] = T(_train_x) - xi;
        residuals[1] = T(_train_y) - yi;

        // --- Forcing scale to be close to 1
        residuals[2] = T(_reg_factor) * (a*a + b*b - T(1));
        //residuals[3] = T(_reg_factor) * (ap*ap + bp*bp - T(1));

        return true;
    }

    double _query_x, _query_y;
    double _train_x, _train_y;
    double _reg_factor;
};

}
#endif // MOSAICADJUSTER_H
