#include <math.h>
#include "bimos/motionest/SimilarityEstimator2D.h"

namespace bimos
{

#define THIS SimilarityEstimator2D

THIS::THIS(double delta) : ParameterEstimator<PointPair2D,double>(2), deltaSquared(delta*delta) {}

/*
 * Normalize points (normalization proposed in "Multiple View Geometry in Computer Vision" by Hartley - Zisserman)
 */

double inline SQF(double x) { return x * x; }

void THIS::normalizePoints(std::vector<PointPair2D *> &data)
{
    if (ndata.size()) ndata.clear();

    int n = data.size();
    Pc.x = 0.0; Pc.y = 0.0;
    Qc.x = 0.0; Qc.y = 0.0;
    for(int k = 0; k < n; k++) {
        Pc.x += data[k]->second.x;  Pc.y += data[k]->second.y;
        Qc.x += data[k]->first.x; Qc.y += data[k]->first.y;
    }
    Pc.x /= n; Pc.y /= n;
    Qc.x /= n; Qc.y /= n;
    Ps = 0.0, Qs = 0.0;
    for(int k = 0; k < n; k++) {
        PointPair2D pp;
        pp.first.x = data[k]->second.x - Pc.x;
        pp.first.y = data[k]->second.y - Pc.y;
        pp.second.x = data[k]->first.x - Qc.x;
        pp.second.y = data[k]->first.y - Qc.y;
        ndata.push_back(pp);
        Ps += sqrt(SQF(pp.second.x)  + SQF(pp.second.y));
        Qs += sqrt(SQF(pp.first.x) + SQF(pp.first.y));
    }
    double sqrt2_n = sqrt(2.0) * n;
    Ps = sqrt2_n / Ps;
    Qs = sqrt2_n / Qs;
    for(int k = 0; k < n; k++) {
        ndata[k].second.x *= Ps;  ndata[k].second.y *= Ps;
        ndata[k].first.x *= Qs; ndata[k].first.y *= Qs;
    }
}

/*
 * Denormalizes similarity transform: T_Q^{-1} Hbar T_P, where T_P and T_Q are the normalization transforms
 *
 *       | Ps 0  -Ps * Pc,x |
 * T_P = | 0  Ps -Ps * Pc,y |
 *       | 0  0   1         |
 *
 *
 *            | 1/Qs 0    Qc,x |
 * T_Q^{-1} = | 0    1/Qs Qc,y |
 *            | 0    0    1    |
 *
 */

void THIS::denormalizeSimilarity(void)
{
    double scale_ratio = Ps / Qs;
    s_cos_phi *= scale_ratio;
    s_sin_phi *= scale_ratio;
    tx = Qc.x + tx/Qs - (Pc.x * s_cos_phi - Pc.y * s_sin_phi); // tx = 0 after normalization?
    ty = Qc.y + ty/Qs - (Pc.x * s_sin_phi + Pc.y * s_cos_phi); // ty = 0 after normalization?
}

/*
 * Compute the similarity parameters [s*cos_phi, s*sin_phi, tx, ty]
 */
void THIS::estimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters)
{
    if(data.size() < this->minForEstimate) {
        parameters.clear();
        return;
    }

    normalizePoints(data);

    Point2D *P1 = &ndata[0].second,  *P2 = &ndata[1].second;
    Point2D *Q1 = &ndata[0].first, *Q2 = &ndata[1].first;
    double dPx = P1->x - P2->x;
    double dPy = P1->y - P2->y;
    double dQx = Q1->x - Q2->x;
    double dQy = Q2->y - Q2->y;

    double den = dPx * dPx + dPy * dPy;
    s_cos_phi = (dPx * dQx + dPy * dQy) / den;
    s_sin_phi = (dPx * dQy - dPy * dQx) / den;
    tx = Q2->x - P2->x * s_cos_phi + P2->y * s_sin_phi;
    ty = Q2->y - P2->x * s_sin_phi - P2->y * s_cos_phi;

    denormalizeSimilarity();

    parameters.clear();
    parameters.push_back(s_cos_phi);
    parameters.push_back(s_sin_phi);
    parameters.push_back(tx);
    parameters.push_back(ty);

    //printf("\n\n");
}

/*
 * Compute the similarity parameters [s*cos_phi, s*sin_phi, tx, ty]
 */
void THIS::leastSquaresEstimate(std::vector<PointPair2D *> &data, std::vector<double> &parameters)
{
    int n = data.size();
    if (n < this->minForEstimate) {
        parameters.clear();
        return;
    }

    normalizePoints(data);

    double cx = 0.0, sx = 0.0, xx = 0.0;
    for(int k = 0; k < n; k++) {
        Point2D *P = &ndata[k].second, *Q = &ndata[k].first;
        cx += P->x * Q->x + P->y * Q->y;
        sx += P->x * Q->y - P->y * Q->x;
        xx += P->x * P->x + P->y * P->y;
    }
    s_cos_phi = cx / xx;
    s_sin_phi = sx / xx;
    tx = 0.0;
    ty = 0.0;

    denormalizeSimilarity();

    parameters.clear();
    parameters.push_back(s_cos_phi);
    parameters.push_back(s_sin_phi);
    parameters.push_back(tx);
    parameters.push_back(ty);
}

/*
 * Given the similarity parameters [s*cos_phi, s*sin_phi, tx, ty] check if
 * ((HP - Q)^2 + (H^(-1)Q - P)^2)/2 < delta^2
 */
bool THIS::agree(std::vector<double> &parameters, PointPair2D &data, double* distance)
{
    double s_cos_phi = parameters[0];
    double s_sin_phi = parameters[1];
    double tx = parameters[2];
    double ty = parameters[3];

    double Px = data.second.x,  Py = data.second.y;
    double Qx = data.first.x, Qy = data.first.y;
    double PQx = Qx - (Px * s_cos_phi - Py * s_sin_phi + tx);
    double PQy = Qy - (Px * s_sin_phi + Py * s_cos_phi + ty);
    double s = SQF(s_cos_phi) + SQF(s_sin_phi);
    double dx = Qx - tx, dy = Qy - ty;
    double QPx = Px - ( dx * s_cos_phi + dy * s_sin_phi) / s;
    double QPy = Py - (-dx * s_sin_phi + dy * s_cos_phi) / s;

    double distanceSquared = ( (SQF(PQx) + SQF(PQy)) + (SQF(QPx) + SQF(QPy)) ) / 2;

    //printf("(%f^2 + %f^2 + %f^2 + %f^2)/2 = %f < %f\n", PQx, PQy, QPx, QPy, signedDistanceSquared, deltaSquared);

    bool ok = distanceSquared < deltaSquared;
    //printf("%d",ok);

    if (distance) (*distance) = (ok) ? distanceSquared : deltaSquared;

    return ok;
}

void THIS::debugTest(std::ostream &out)
{
}

}
