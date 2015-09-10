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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <opencv2/opencv.hpp>

namespace bimos
{

enum TransformType
{
    SIMILARITY,
    AFFINE,
    PERSPECTIVE,
    ROTATIONAL
};

/**  
Similarity Transformation.
4 DoF.
Format:
    a   -b   c
    b    a   d
    0    0   1
where:
    a = s * cos(theta)
    b = s * sin(theta)
    c = tx
    d = ty

Affine Transformation
6 DoF
Format
    a    b    e
    c    d    f
    0    0    1

Perspective Transformation
8 DoF
Format
    a    b    e
    c    d    f
    g    h    1
*/

class Transform
{
public:
    Transform(const TransformType& _type = SIMILARITY);
    Transform(const cv::Mat_<double>& _H, const TransformType& _type = SIMILARITY);
    Transform(const Transform& other);
    ~Transform();

    Transform& operator=(const Transform& other);
    Transform operator*(const Transform& other);

    void decomposeTransformation(std::vector<double>& _params);
    void updateHomography();
    Transform inv();
    std::string toString();

    //  Homography
    cv::Mat_<double> H;

    // Transformation parameters
    // Similarity (a, b, c, d) = (scos(th), ssin(th), tx, ty)
    // Affine (a, b, c, d, e, f)
    // Perpective (a, b, c, d, e, f, g)
    double* params;

    // Transformation type
    TransformType type;
};

}

#endif // TRANSFORM_H
