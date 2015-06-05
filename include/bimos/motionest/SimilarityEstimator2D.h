#ifndef _SIMILARITY_ESTIMATOR_2D_H_
#define _SIMILARITY_ESTIMATOR_2D_H_

#include <utility> // for 'pair' class

#include "bimos/motionest/ParameterEstimator.h"
#include "bimos/motionest/Point2D.h"

namespace bimos
{

/**
 * This class estimates the parameters of a similarity transformation.
 * A similarity is represented as:
 *
 *     | s*ct -s*st tx |
 * H = | s*st +s*ct ty |
 *     | 0     0    1  |
 *
 * All point pairs P and Q which meet the similarity satisfy ((HP - Q)^2 + (H^(-1)Q - P)^2) = 0.
 *
 * Author: Alberto Ortiz
 */

class SimilarityEstimator2D : public ParameterEstimator<PointPair2D,double> {
public:

    SimilarityEstimator2D(double delta);

    /**
     * Compute the similarity defined by the given 2 pairs of data points.
     * @param data A vector containing two pairs of 2D points.
     * @param parameters This vector is cleared and then filled with the computed parameters.
     *        The parameters of the similarity transformation for these points are
     *				[s*cos_phi, s*sin_phi, tx, ty]
     *        If the vector contains less than 2 pairs of points then the resulting parameters
     *        vector is empty (size = 0).
     */
    virtual void estimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters);

    /**
     * Compute a least squares estimate of the similarity defined by the given points.
     *
     * @param data The similarity should minimize the least squares error to these points.
     * @param parameters This vector is cleared and then filled with the computed parameters.
     *        The parameters of the similarity transformation for these points are
     *				[s*cos_phi, s*sin_phi, tx, ty]
     *        If the vector contains less than 2 pairs of points then the resulting parameters
     *        vector is empty (size = 0).
     */
    virtual void leastSquaresEstimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters);

    /**
     * Return true if the squared symmetric reprojection error for the similarity defined by the parameters and the
     * given pair of points is smaller than 'delta' (see constructor).
     * @param parameters The similarity parameters [s*cos_phi, s*sin_phi, tx, ty].
     * @param data The pair of points.
     */
    virtual bool agree(std::vector<double> &parameters, PointPair2D &data, double* distance = 0);

    /**
     * Test the class methods, output to specified stream.
     */
    static void debugTest(std::ostream &out);

private:

    double deltaSquared; //!< Given a similarity H and a pair of points <P,Q>, if ((HP - Q)^2 + (H^(-1)Q - P)^2)/2 < delta^2 then the pair obeys the similarity

    void normalizePoints(std::vector<PointPair2D *> &data);	//!< Normalize 'data' and leave normalized points in member 'ndata'
    void denormalizeSimilarity(void);											//!< Denormalize similarity stored in members

    double Ps;	//!< Last normalization scale for P points
    double Qs;	//!< Last normalization scale for Q points
    Point2D Pc;	//!< Last normalization centroid for P points
    Point2D Qc;	//!< Last normalization centroid for Q points

    double s_cos_phi;	//!< Similarity transform: s * cos(phi)
    double s_sin_phi;	//!< Similarity transform: s * sin(phi)
    double tx;				//!< Similarity transform: tx
    double ty;				//!< Similarity transform: ty

    std::vector<PointPair2D> ndata;	//!< Normalized list of point pairs

};

}
#endif //_SIMILARITY_ESTIMATOR_2D_H_
