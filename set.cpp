//
// Created by David Woller on 5.10.18.
//


#include "set.h"
#include <utility>
#include <chrono>
#include <iostream>
#include <random>

using namespace glns;

Set::Set() {
    std::mt19937 generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    rgb = {distribution(generator), distribution(generator), distribution(generator)};
}

Set::Set(std::vector<Vertex> vertices, int id) : vertices(std::move(vertices)), id(id) {
    std::mt19937 generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    rgb = {distribution(generator), distribution(generator), distribution(generator)};
}
