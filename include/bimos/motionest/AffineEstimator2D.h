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

#ifndef _AFFINE_ESTIMATOR_2D_H_
#define _AFFINE_ESTIMATOR_2D_H_

#include <utility> // for 'pair' class

#include "bimos/motionest/ParameterEstimator.h"
#include "bimos/motionest/Point2D.h"

namespace bimos
{

/**
 * This class estimates the parameters of an affine transformation.
 * It is is represented as:
 *
 *     | a  b  e |
 * H = | c  d  f |
 *     | 0  0  1 |
 *
 * All point pairs P and Q which meet the transformation satisfy ((HP - Q)^2 + (H^(-1)Q - P)^2) = 0.
 *
 * Author: Emilio Garcia-Fidalgo
 */

class AffineEstimator2D : public ParameterEstimator<PointPair2D,double> {
public:

    AffineEstimator2D(double delta);

    /**
     * Compute the similarity defined by the given 3 pairs of data points.
     * @param data A vector containing three pairs of matched 2D points.
     * @param parameters This vector is cleared and then filled with the computed parameters.
     *        The parameters of the similarity transformation for these points are
     *				[a, b, c, d, e, f]
     *        If the vector contains less than 3 pairs of points then the resulting parameters
     *        vector is empty (size = 0).
     */
    virtual void estimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters);

    /**
     * Compute a least squares estimate of the similarity defined by the given points.
     *
     * @param data The similarity should minimize the least squares error to these points.
     * @param parameters This vector is cleared and then filled with the computed parameters.
     *        The parameters of the similarity transformation for these points are
     *				[a, b, c, d, e, f]
     *        If the vector contains less than 3 pairs of points then the resulting parameters
     *        vector is empty (size = 0).
     */
    virtual void leastSquaresEstimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters);

    /**
     * Return true if the squared symmetric reprojection error for the affine transformation defined by the parameters and the
     * given pair of points is smaller than 'delta' (see constructor).
     * @param parameters The similarity parameters [a, b, c, d, e, f].
     * @param data The pair of points.
     */
    virtual bool agree(std::vector<double> &parameters, PointPair2D &data, double* distance = 0);

    /**
     * Test the class methods, output to specified stream.
     */
    static void debugTest(std::ostream &out);

private:
    double deltaSquared; //!< Given an affine transform H and a pair of points <P,Q>, if ((HP - Q)^2 + (H^(-1)Q - P)^2)/2 < delta^2 then the pair obeys the similarity
};

}
#endif //_AFFINE_ESTIMATOR_2D_H_
