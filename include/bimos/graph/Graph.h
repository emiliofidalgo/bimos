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

#ifndef GRAPH_H
#define GRAPH_H

#include <limits>
#include <set>
#include <vector>

namespace bimos
{

typedef int Vertex;
typedef double Weight;

const Weight MAX_WEIGHT = std::numeric_limits<double>::infinity();

struct Neighbor
{
    Vertex target;
    Weight weight;

    Neighbor(Vertex arg_target, Weight arg_weight) :
        target(arg_target), weight(arg_weight)
    {}
};

struct Graph
{
    std::vector<std::vector<Neighbor> > adjlist;

    std::vector<Vertex> previous;
    std::vector<Weight> min_distance;

    Graph();
    Vertex addVertex();
    void addEdge(Vertex a, Vertex b, Weight w);    

    // Dijkstra
    void DijkstraComputePaths(Vertex source);
    std::vector<Vertex> DijkstraGetShortestPathTo(Vertex vertex);
};

}

#endif // GRAPH_H
