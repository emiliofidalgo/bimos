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

#include <bimos/optim/MosaicAdjuster.h>

namespace bimos
{

/**
 * @brief Default class constructor.
 */
MosaicAdjuster::MosaicAdjuster(const double _rfactor) :
    _problem(0),
    rfactor(_rfactor)
{
    _problem = new ceres::Problem();
}

/**
 * @brief Default destructor.
 */
MosaicAdjuster::~MosaicAdjuster()
{
    boost::mutex::scoped_lock lock(mutex_adjuster);
    delete _problem;
}

/**
 * @brief Adds matchings between two KFs as constraints to the optimization problem.
 * @param node_prev Origin KF.
 * @param node Destination KF.
 * @param matches Set of matchings between the KFs.
 */
void MosaicAdjuster::addConstraints(Keyframe* kf_prev, Keyframe* kf, const std::vector<cv::DMatch>& matches)
{
    boost::mutex::scoped_lock lock(mutex_adjuster);

    // Iterating for each match
    for (unsigned match_ind = 0; match_ind < matches.size(); match_ind++)
    {
        int query_id = matches[match_ind].queryIdx;
        int train_id = matches[match_ind].trainIdx;

        cv::Point2f qpoint = kf->image->kps[query_id].pt;
        cv::Point2f tpoint = kf_prev->image->kps[train_id].pt;

        AdjusterCostFunctorSim* hcfunctor = new AdjusterCostFunctorSim(qpoint, tpoint, rfactor);
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<AdjusterCostFunctorSim, 3, 4, 4>(hcfunctor);
        _problem->AddResidualBlock(cost_function, new ceres::HuberLoss(0.2), kf_prev->trans.params, kf->trans.params);
    }
}

/**
 * @brief Adjusts the positions of the graph.
 * @param options Ceres optimizer options.
 * @param summary Summary about the optimization process.
 */
void MosaicAdjuster::adjust(const ceres::Solver::Options& options, ceres::Solver::Summary& summary)
{
    boost::mutex::scoped_lock lock(mutex_adjuster);
    ceres::Solver::Summary sum;
    ceres::Solve(options, _problem, &sum);
    summary = sum;
}

/**
 * @brief Restarts the Ceres problem
 */
void MosaicAdjuster::reset()
{
    boost::mutex::scoped_lock lock(mutex_adjuster);
    if (_problem)
    {
        delete _problem;
    }

    _problem = new ceres::Problem();
}

}
