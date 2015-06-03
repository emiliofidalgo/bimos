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

#include "bimos/util/Transform.h"

namespace bimos
{

/**
 * @brief Default class constructor
 * @param _type Transform type
 */
Transform::Transform(const TransformType& _type)
{
    type = _type;
    H = cv::Mat::eye(3, 3, CV_64F);
    switch (type)
    {
    case AFFINE:
        params = new double[6];
        params[0] = 1.0; // a
        params[1] = 0.0; // b
        params[2] = 0.0; // c
        params[3] = 1.0; // d
        params[4] = 0.0; // e
        params[5] = 0.0; // f
        break;
    case PERSPECTIVE:
        params = new double[8];
        params[0] = 1.0; // a
        params[1] = 0.0; // b
        params[2] = 0.0; // c
        params[3] = 1.0; // d
        params[4] = 0.0; // e
        params[5] = 0.0; // f
        params[6] = 0.0; // g
        params[7] = 0.0; // h
        break;
    case SIMILARITY:
    default:
        params = new double[4];
        params[0] = 1.0; // scos
        params[1] = 0.0; // ssin
        params[2] = 0.0; // tx
        params[3] = 0.0; // ty
        break;
    }
}

/**
 * @brief Class constructor
 * @param _type Transform type
 * @param _H Homography
 */
Transform::Transform(const cv::Mat_<double>& _H, const TransformType& _type)
{
    type = _type;
    H.release();
    _H.copyTo(H);
    switch (type)
    {
    case AFFINE:
        params = new double[6];
        params[0] = H(0, 0); // a
        params[1] = H(0, 1); // b
        params[2] = H(1, 0); // c
        params[3] = H(1, 1); // d
        params[4] = H(0, 2); // e
        params[5] = H(1, 2); // f
        break;
    case PERSPECTIVE:
        params = new double[8];
        params[0] = H(0, 0); // a
        params[1] = H(0, 1); // b
        params[2] = H(1, 0); // c
        params[3] = H(1, 1); // d
        params[4] = H(0, 2); // e
        params[5] = H(1, 2); // f
        params[6] = H(2, 0); // g
        params[7] = H(2, 1); // h
        break;
    case SIMILARITY:
    default:
        params = new double[4];
        params[0] = H(0, 0); // scos
        params[1] = H(1, 0); // ssin
        params[2] = H(0, 2); // tx
        params[3] = H(1, 2); // ty
        break;
    }
}

/**
 * @brief Copy constructor
 * @param other The other transformation
 */
Transform::Transform(const Transform& other)
{
    type = other.type;
    H.release();
    other.H.copyTo(H);
    switch (type)
    {
    case AFFINE:
        params = new double[6];
        params[0] = other.params[0]; // a
        params[1] = other.params[1]; // b
        params[2] = other.params[2]; // c
        params[3] = other.params[3]; // d
        params[4] = other.params[4]; // e
        params[5] = other.params[5]; // f
        break;
    case PERSPECTIVE:
        params = new double[8];
        params[0] = other.params[0]; // a
        params[1] = other.params[1]; // b
        params[2] = other.params[2]; // c
        params[3] = other.params[3]; // d
        params[4] = other.params[4]; // e
        params[5] = other.params[5]; // f
        params[6] = other.params[6]; // g
        params[7] = other.params[7]; // h
        break;
    case SIMILARITY:
    default:
        params = new double[4];
        params[0] = other.params[0]; // scos
        params[1] = other.params[1]; // ssin
        params[2] = other.params[2]; // tx
        params[3] = other.params[3]; // ty
        break;
    }
}

/**
 * @brief Default destructor
 */
Transform::~Transform()
{
    delete params;
}

/**
 * @brief Transform::operator =
 * @param other The other transformation
 * @return
 */
Transform& Transform::operator=(const Transform& other)
{
    assert(type == other.type);

    type = other.type;
    H.release();
    other.H.copyTo(H);
    switch (type)
    {
    case AFFINE:
        params[0] = other.params[0]; // a
        params[1] = other.params[1]; // b
        params[2] = other.params[2]; // c
        params[3] = other.params[3]; // d
        params[4] = other.params[4]; // e
        params[5] = other.params[5]; // f
        break;
    case PERSPECTIVE:
        params[0] = other.params[0]; // a
        params[1] = other.params[1]; // b
        params[2] = other.params[2]; // c
        params[3] = other.params[3]; // d
        params[4] = other.params[4]; // e
        params[5] = other.params[5]; // f
        params[6] = other.params[6]; // g
        params[7] = other.params[7]; // h
        break;
    case SIMILARITY:
    default:
        params[0] = other.params[0]; // scos
        params[1] = other.params[1]; // ssin
        params[2] = other.params[2]; // tx
        params[3] = other.params[3]; // ty
        break;
    }

    return *this;
}

/**
 * @brief Transform::operator *
 * @param other The other transformation
 * @return
 */
Transform Transform::operator*(const Transform& other)
{
    assert(type == other.type);

    cv::Mat_<double> Ht = H * other.H;

    Transform result(Ht, type);
    return result;
}

/**
 * @brief Decompose the homography in parameters
 * @param _params The output parameters
 */
void Transform::decomposeTransformation(std::vector<double>& _params)
{
    switch (type)
    {
    case AFFINE:
    case PERSPECTIVE:
        _params.push_back(params[4]);
        _params.push_back(params[5]);
        break;
    case SIMILARITY:
    default:
        double tx = H(0, 2); _params.push_back(tx);
        double ty = H(1, 2); _params.push_back(ty);
        double theta = atan2(H(1, 0), H(0, 0)); _params.push_back(theta);
        double s = sqrt(H(0, 0) * H(0, 0) + H(1, 0) * H(1, 0)); _params.push_back(s);
        break;
    }
}

/**
 * @brief Update the homography using the values stored in params
 */
void Transform::updateHomography()
{
    switch (type)
    {
    case AFFINE:
        H(0, 0) = params[0];
        H(0, 1) = params[1];
        H(1, 0) = params[2];
        H(1, 1) = params[3];
        H(0, 2) = params[4];
        H(1, 2) = params[5];
        break;
    case PERSPECTIVE:
        H(0, 0) = params[0];
        H(0, 1) = params[1];
        H(1, 0) = params[2];
        H(1, 1) = params[3];
        H(0, 2) = params[4];
        H(1, 2) = params[5];
        H(2, 0) = params[6];
        H(2, 1) = params[7];
        break;
    case SIMILARITY:
    default:
        H(0, 0) = params[0];
        H(1, 1) = params[0];
        H(1, 0) = params[1];
        H(0, 1) = -params[1];
        H(0, 2) = params[2];
        H(1, 2) = params[3];
        break;
    }
}

/**
 * @brief Computes the inverse transformation
 * @return The inverse transformation
 */
Transform Transform::inv()
{
    switch (type)
    {
    case AFFINE:
    {
        double a = params[0];
        double b = params[1];
        double c = params[2];
        double d = params[3];
        double e = params[4];
        double f = params[5];

        // Inverting the matrix
        double den = a * d - b * c;
        double ainv = d / den;
        double binv = -b / den;
        double cinv = -c / den;
        double dinv = a / den;
        double einv = (b * f - d * e) / den;
        double finv = -(a * f - c * e) / den;

        Transform inverse(AFFINE);
        inverse.params[0] = ainv;
        inverse.params[1] = binv;
        inverse.params[2] = cinv;
        inverse.params[3] = dinv;
        inverse.params[4] = einv;
        inverse.params[5] = finv;
        inverse.updateHomography();

        return inverse;
    }
    case PERSPECTIVE:
    {
        double a = params[0];
        double b = params[1];
        double c = params[2];
        double d = params[3];
        double e = params[4];
        double f = params[5];
        double g = params[6];
        double h = params[7];

        // Inverting the matrix
        double den = a * d - b * c - a * f * h + b * f * g + c * e * h - d * e * g;
        double h33 = (a * d - b * c) / den;
        double ainv = (d - f * h) / den;
        ainv /= h33;
        double binv = -(b - e * h) / den;
        binv /= h33;
        double cinv = -(c - f * g) / den;
        cinv /= h33;
        double dinv = (a - e * g) / den;
        dinv /= h33;
        double einv = (b * f - d * e) / den;
        einv /= h33;
        double finv = -(a * f - c * e) / den;
        finv /= h33;
        double ginv = (c * h - d * g) / den;
        ginv /= h33;
        double hinv = -(a * h - b * g) / den;
        hinv /= h33;

        Transform inverse(PERSPECTIVE);
        inverse.params[0] = ainv;
        inverse.params[1] = binv;
        inverse.params[2] = cinv;
        inverse.params[3] = dinv;
        inverse.params[4] = einv;
        inverse.params[5] = finv;
        inverse.params[6] = ginv;
        inverse.params[7] = hinv;
        inverse.updateHomography();

        return inverse;
    }
    case SIMILARITY:
    default:
    {
        double a = params[0];
        double b = params[1];
        double c = params[2];
        double d = params[3];

        // Inverting the matrix
        double den = a * a + b * b;
        double ainv = a / den;
        double binv = -b / den;
        double cinv = -(a * c + b * d) / den;
        double dinv = -(a * d - b * c) / den;

        Transform inverse(SIMILARITY);
        inverse.params[0] = ainv;
        inverse.params[1] = binv;
        inverse.params[2] = cinv;
        inverse.params[3] = dinv;
        inverse.updateHomography();

        return inverse;
    }
    }
}

/**
 * @brief Returns the string representation of the transformation
 * @return The string representing the transformation
 */
std::string Transform::toString()
{
    std::stringstream os;

    switch (type)
    {
    case PERSPECTIVE:
    case AFFINE:
        os << "---" << std::endl;
        os << H << std::endl;
        os << "---" << std::endl;
        break;
    case SIMILARITY:
    default:
        os << "---" << std::endl;
        os << H << std::endl;
        std::vector<double> params;
        decomposeTransformation(params);
        os << "- Tx: " << params[0] << std::endl << "- Ty: " << params[1] << std::endl << "- Theta: " << params[2] << std::endl << "- Scale: " << params[3] << std::endl;
        os << "---" << std::endl;
    }

    return os.str();
}

}
