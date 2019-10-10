//
// Created by David Woller on 2.10.18.
//

#ifndef GLNS_EDGE_H
#define GLNS_EDGE_H


#include "vertex.h"

/*
 * This class represents edge in Rattled grid GTSP problem definition.
 */
namespace glns {

class Edge {
public:
    Edge() = default;;
    Edge(Vertex from, Vertex to, double weigth, int id) : from(from), to(to), weight(weigth), id(id) {};
    void draw(const Cairo::RefPtr<Cairo::Context>& cr);

    Vertex from;
    Vertex to;
    double weight;
    int id; // edges are indexed from 0, id should correspond to position in vector "edges"
};

}

#endif //GLNS_EDGE_H
