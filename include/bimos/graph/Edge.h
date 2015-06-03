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

#ifndef MEDGE_H
#define MEDGE_H

#include "bimos/util/Transform.h"

namespace bimos
{

struct Edge
{
    Edge() :
        ori(0),
        dest(0),
        weight(0.0)
    {
    }

    Edge(const cv::Mat_<double>& trans) :
        ori(0),
        dest(0),
        weight(0.0),
        trans(trans)
    {
    }

    Edge(const Edge& other)
    {
        ori = other.ori;
        dest = other.dest;
        weight = other.weight;
        trans = other.trans;
    }

    Edge(const int o, const int d, const double w, const cv::Mat_<double>& t) :
        ori(o),
        dest(d),
        weight(w),
        trans(t)
    {
    }

    Edge& operator=(const Edge& other)
    {
        ori = other.ori;
        dest = other.dest;
        weight = other.weight;
        trans = other.trans;
    }

    int ori, dest;
    double weight;
    Transform trans;
};

}

#endif // MEDGE_H
