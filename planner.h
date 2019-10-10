//
// Created by David Woller on 6.10.18.
//

#ifndef GLNS_PLANNER_H
#define GLNS_PLANNER_H

#include <map>
#include "tour.h"
#include "heuristic.h"

namespace glns {

class Canvas;

class Planner {
public:
    Planner();
    void entryPoint(Canvas *caller, int argc, char *argv[]);

    void precomputeSetVertexDistances();
    void initHeuristics();

    Tour initRandomTour();
    Tour initRandomInsertionTour();
    Tour initPartialTour(int length);

    Vertex getRandomVertex(std::vector<Vertex> vertices);
    Tour removeVertexFromTour(Tour tour, Vertex vertex);
    double getTourWeight(Tour tour);

    Set unifiedSetSelection(Tour partialTour, double lambda);
    Set cheapestSetSelection(Tour partialTour);
    Tour unifiedInsertion(Tour partialTour, double lambda, double my);

    Tour segmentRemoval(Tour tour, int N_r);
    Tour distanceRemoval(Tour tour, int N_r, double lambda);
    Tour worstRemoval(Tour tour, int N_r, double lambda);
    Tour removalFramework(Tour tour, double lambda);
    Tour removeInsert(Tour current, std::string phase);

    Heuristic * selectInsertionHeuristic(std::string phase);
    Heuristic * selectRemovalHeuristic(std::string phase);
    void updateHeuristicsWeights(double epsilon);
    void printWeights();

    bool acceptTrial(double trialCost, double currentCost, double temperature);
    bool acceptTrialNoParam(double trialCost, double currentCost, double probAccept);

    Tour reOpt(Tour tour);
    Tour moveOpt(Tour tour, int NMove);
    Tour optCycle(Tour tour, int NMove);

    void animationDemo(Canvas *caller);
    void partialTourDemo(Canvas *caller);
    void unifiedInsertionDemo(Canvas *caller);
    void removalHeuristicsDemo(Canvas *caller);
    void givenTourDisplayDemo(Canvas *caller);
    void reOptDemo(Canvas *caller);
    void moveOptDemo(Canvas *caller);

    Tour solver(Canvas *caller, std::string mode, double maxTime, double tourBudget);

private:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Set> sets;
    std::vector<std::vector<Edge> > edgeMatrix;
    std::map<std::pair<int, int>, double> setVertexDistances; // (set.id, vertex.id) -> minDistance
    std::vector<Heuristic> insertionHeuristics;
    std::vector<Heuristic> removalHeuristics;

};

}

#endif //GLNS_PLANNER_H
