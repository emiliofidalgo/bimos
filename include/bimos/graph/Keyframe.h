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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <omp.h>

#include <bimos/util/Image.h>
#include <bimos/util/Transform.h>

namespace bimos
{

struct Keyframe
{
    Keyframe(Image* img) :
        image(img),
        init_time(omp_get_wtime())
    {
    }

    Keyframe(Image* img, const cv::Mat& transform) :
        image(img),
        trans(transform)
    {
    }

    int id;
    Image* image;
    Transform trans;

    // Timing variables
    double init_time;
    double end_time;
};

}

#endif // KEYFRAME_H
