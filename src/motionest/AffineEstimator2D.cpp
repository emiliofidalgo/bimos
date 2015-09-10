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

#include <math.h>
#include "bimos/motionest/AffineEstimator2D.h"

namespace bimos
{

#define THIS AffineEstimator2D

THIS::THIS(double delta) : ParameterEstimator<PointPair2D,double>(3), deltaSquared(delta*delta) {}

double inline SQF(double x) { return x * x; }

/*
 * Compute the similarity parameters [a, b, c, d, e, f]
 */
void THIS::estimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters)
{
    if(data.size() < this->minForEstimate)
    {
        parameters.clear();
        return;
    }

    Point2D T1 = data[0]->first,  T2 = data[1]->first,  T3 = data[2]->first;
    Point2D Q1 = data[0]->second, Q2 = data[1]->second, Q3 = data[2]->second;

    double t1x = T1.x, t1y = T1.y;
    double t2x = T2.x, t2y = T2.y;
    double t3x = T3.x, t3y = T3.y;

    double q1x = Q1.x, q1y = Q1.y;
    double q2x = Q2.x, q2y = Q2.y;
    double q3x = Q3.x, q3y = Q3.y;

    double den = q1x * q2y - q2x * q1y - q1x * q3y + q3x * q1y + q2x * q3y - q3x * q2y;
    double a = (t1x*(q2y-q3y))/(den)-(t2x*(q1y-q3y))/(den)+(t3x*(q1y-q2y))/(den);
    double b = -(t1x*(q2x-q3x))/(den)+(t2x*(q1x-q3x))/(den)-(t3x*(q1x-q2x))/(den);
    double c = (t1y*(q2y-q3y))/(den)-(t2y*(q1y-q3y))/(den)+(t3y*(q1y-q2y))/(den);
    double d = -(t1y*(q2x-q3x))/(den)+(t2y*(q1x-q3x))/(den)-(t3y*(q1x-q2x))/(den);
    double e = (t3x*(q1x*q2y-q2x*q1y))/(den)-(t2x*(q1x*q3y-q3x*q1y))/(den)+(t1x*(q2x*q3y-q3x*q2y))/(den);
    double f = (t3y*(q1x*q2y-q2x*q1y))/(den)-(t2y*(q1x*q3y-q3x*q1y))/(den)+(t1y*(q2x*q3y-q3x*q2y))/(den);

    parameters.clear();
    parameters.push_back(a);
    parameters.push_back(b);
    parameters.push_back(c);
    parameters.push_back(d);
    parameters.push_back(e);
    parameters.push_back(f);
}

/*
 * Compute the similarity parameters [a, b, c, d, e, f]
 */
void THIS::leastSquaresEstimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters)
{
    int n = data.size();
    if (n < this->minForEstimate)
    {
        parameters.clear();
        return;
    }

    double s1 = 0.0, s2 = 0.0, s3 = 0.0, s4 = 0.0, s5 = 0.0;
    double w1 = 0.0, w2 = 0.0, w3 = 0.0, w4 = 0.0, w5 = 0.0, w6 = 0.0;
    for(int k = 0; k < n; k++)
    {
        // Getting point values
        Point2D T = data[k]->first;
        Point2D Q = data[k]->second;
        double tx = T.x, ty = T.y;
        double qx = Q.x, qy = Q.y;

        // Getting values for A' * A
        s1 += SQF(qx);
        s2 += SQF(qy);
        s3 += qx * qy;
        s4 += qx;
        s5 += qy;

        // Getting values for A' * t
        w1 += qx * tx;
        w2 += qy * tx;
        w3 += qx * ty;
        w4 += qy * ty;
        w5 += tx;
        w6 += ty;
    }

    // Computing inv(A' * A)
    double invd = n * SQF(s3) - 2 * s3 * s4 * s5 + s2 * SQF(s4) + s1 * SQF(s5) - n * s1 * s2;
    double k1 = -(-SQF(s5) + n * s2);
    double k2 = n * s3 - s4 * s5;
    double k3 = s2 * s4 - s3 * s5;
    double k4 = -(-SQF(s4) + n * s1);
    double k5 = s1 * s5 - s3 * s4;
    double k6 = -(-SQF(s3) + s1 * s2);

    // Computing homography parameters
    double a = (k1 * w1 + k2 * w2 + k3 * w5) / invd;
    double b = (k2 * w1 + k4 * w2 + k5 * w5) / invd;
    double c = (k1 * w3 + k2 * w4 + k3 * w6) / invd;
    double d = (k2 * w3 + k4 * w4 + k5 * w6) / invd;
    double e = (k3 * w1 + k5 * w2 + k6 * w5) / invd;
    double f = (k3 * w3 + k5 * w4 + k6 * w6) / invd;

    parameters.clear();
    parameters.push_back(a);
    parameters.push_back(b);
    parameters.push_back(c);
    parameters.push_back(d);
    parameters.push_back(e);
    parameters.push_back(f);
}

/*
 * Given the similarity parameters [a, b, c, d, e, f] check if
 * ((HP - Q)^2 + (H^(-1)Q - P)^2)/2 < delta^2
 */
bool THIS::agree(std::vector<double> &parameters, PointPair2D &data, double* distance)
{
    double a = parameters[0];
    double b = parameters[1];
    double c = parameters[2];
    double d = parameters[3];
    double e = parameters[4];
    double f = parameters[5];

    // Inverting the matrix
    double den = a * d - b * c;
    double ainv = d / den;
    double binv = -b / den;
    double cinv = -c / den;
    double dinv = a / den;
    double einv = (b * f - d * e) / den;
    double finv = -(a * f - c * e) / den;

    // Getting point values
    Point2D T = data.first;
    Point2D Q = data.second;
    double tx = T.x, ty = T.y;
    double qx = Q.x, qy = Q.y;

    double QTx = tx - (a * qx + b * qy + e);
    double QTy = ty - (c * qx + d * qy + f);
    double TQx = qx - (ainv * tx + binv * ty + einv);
    double TQy = qy - (cinv * tx + dinv * ty + finv);

    double distanceSquared = ( (SQF(QTx) + SQF(QTy)) + (SQF(TQx) + SQF(TQy)) ) / 2;

    bool ok = distanceSquared < deltaSquared;

    if (distance) (*distance) = (ok) ? distanceSquared : deltaSquared;

    return ok;
}

void THIS::debugTest(std::ostream &out)
{
}

}
