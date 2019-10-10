//
// Created by David Woller on 23.10.18.
//

#ifndef GLNS_HEURISTIC_H
#define GLNS_HEURISTIC_H

#include <string>
#include <utility>
#include <map>

namespace glns {

    class Heuristic {
    public:
Heuristic() = default;
Heuristic(std::string name, double lambda, double my);

std::string name;
double lambda;
double my;
std::map<std::string, double> scores;
std::map<std::string, int> counts;
std::map<std::string, double> weights;
    };
}

#endif //GLNS_HEURISTIC_H
