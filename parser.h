//
// Created by David Woller on 18.9.18.
//

#ifndef GLNS_PARSER_H
#define GLNS_PARSER_H

#include <string>
#include <vector>
#include "arc.h"
#include "vertex.h"
#include "edge.h"
#include "set.h"

namespace glns {

class Parser{
public:
    std::vector<std::string> getLines(std::string filename);
    std::vector<std::string> lineToWords(std::string line, char delim);
    bool wordsToArc(double x_R, double y_R, std::vector<std::string> words, Arc &arc);
    void parse2dGtspInstance(std::string filename, std::vector<Vertex> &vertices, std::vector<Edge> &edges, std::vector<Set> &sets, std::vector<std::vector<Edge> > &edgeMatrix);

};
}

#endif //GLNS_PARSER_H
