//
// Created by David Woller on 4.10.18.
//

#ifndef GLNS_SET_H
#define GLNS_SET_H

#include <vector>
#include "vertex.h"

namespace glns {

class Set {
public:
    Set();
    Set(std::vector<Vertex> vertices, int id);

    std::vector<Vertex> vertices;
    int id; // sets are indexed from 0; id should correspond with position in vector "sets"
    std::vector<double> rgb;
    double minDist;
    double minCost;

};

}

#endif //GLNS_SET_H
