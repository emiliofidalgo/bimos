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

#include <bimos/graph/Graph.h>

namespace bimos
{

Graph::Graph()
{
}

/**
 * @brief Adds a vertex to the graph.
 * @return The identifier of the new vertex.
 */
Vertex Graph::addVertex()
{
    Vertex currv = static_cast<int>(adjlist.size());
    std::vector<Neighbor> neighbor_lst;
    adjlist.push_back(neighbor_lst);
    return currv;
}

/**
 * @brief Link two images in the graph.
 * @param a Initial vertex.
 * @param b Goal Vertex.
 * @param w Link weight.
 */
void Graph::addEdge(Vertex a, Vertex b, Weight w)
{
    adjlist[a].push_back(Neighbor(b, w));
}

/**
 * @brief Computes the shortest paths from \a source to the other nodes using the Dijkstra's algorithm.
 * @param source Source node.
 */
void Graph::DijkstraComputePaths(Vertex source)
{
    int n = adjlist.size();
    min_distance.clear();
    min_distance.resize(n, MAX_WEIGHT);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<Weight, Vertex> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty())
    {
        Weight dist = vertex_queue.begin()->first;
        Vertex u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        const std::vector<Neighbor>& neighbors = adjlist[u];
        for (std::vector<Neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            Vertex v = neighbor_iter->target;
            Weight weight = neighbor_iter->weight;
            Weight distance_through_u = dist + weight;
            if (distance_through_u < min_distance[v])
            {
                vertex_queue.erase(std::make_pair(min_distance[v], v));

                min_distance[v] = distance_through_u;
                previous[v] = u;
                vertex_queue.insert(std::make_pair(min_distance[v], v));
            }
        }
    }
}

/**
 * @brief Obtains the shortest path computed using @see Graph::DijkstraComputePaths from the source.
 * @param vertex The goal vertex.
 * @return The shortest path from source to vertex.
 */
std::vector<Vertex> Graph::DijkstraGetShortestPathTo(Vertex vertex)
{
    std::vector<Vertex> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.insert(path.begin(), vertex);
    return path;

}

}
