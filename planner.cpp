//
// Created by David Woller on 6.10.18.
//

#include <iostream>
#include <fstream>

#include <cmath>
#include <chrono>
#include <ctime>
#include <random>

#include "planner.h"
#include "canvas.h"
#include "parser.h"

#include "utils.hpp"
#include "MapDrawer.hpp"
#include "poly_maps.hpp"
#include "triangulation.hpp"
#include "simple_intersection.h"

using namespace glns;

Planner::Planner() = default;

void printMap(pmap::geom::FMap map){
    int index = 0;
    std::cout << "Polygons: " << map.size() << "\n";
    for(auto &poly : map){
        std::cout << "Polygon: " << index++ << "\n";
        for(auto &point : poly){
            std::cout << "Point: " << point.x << " " << point.y << "\n";
        }
    }
}

void writeDataTSPIntoFile(std::string path, int totalPoints, int totalSets, std::vector<pmap::geom::FPoints> &pointsOfPolygons){
    std::ofstream file;
    file.open (path);
    file << "Name : RNG Data\n";
    file << "TYPE : TSP\n";
    file << "DIMENSION : " << totalPoints << "\n";
    file << "GTSP_SETS : " << totalSets << "\n";
    file << "EDGE_WEIGHT_TYPE : EUC_2D\n";
    file << "NODE_COORD_SECTION\n";
    int pointCounter = 1;
    for (int l = 0; l < pointsOfPolygons.size(); ++l) {
        for(auto &point : pointsOfPolygons[l]){
            file << pointCounter++ << " " << point.x << " " << point.y << "\n";
        }
    }
    pointCounter = 1;
    int setCounter = 1;
    file << "GTSP_SET_SECTION\n";
    for (int j = 0; j < pointsOfPolygons.size(); ++j) {
        file << setCounter++ << " ";
        for (int i = 0; i < pointsOfPolygons[j].size(); ++i) {
            file << pointCounter++ << " ";
        }
        file << -1 << "\n";
    }

    file.close();
}

void Planner::entryPoint(glns::Canvas *caller, int argc, char *argv[]) {
    // animationDemo(caller);
    // partialTourDemo(caller);
    // unifiedInsertionDemo(caller);
    // removalHeuristicsDemo(caller);
    // initHeuristics();
    // givenTourDisplayDemo(caller);
    // reOptDemo(caller);
    // moveOptDemo(caller);

    //TODO added code, generate new file
    std::ofstream file;
    pmap::geom::FMap map;
    pmap::loadMap("GeneratedFiles/dataPolygons.txt", map);

    //printMap(map);

    std::vector<pmap::geom::FPoints> pointsCenter;
    std::vector<pmap::geom::FPolygons> triangles;
    std::vector<pmap::geom::FPolygon> polygons;
    int totalNumOfPoints = 0;

    for (int i = 1; i < map.size(); ++i) {
        pmap::geom::FPolygon pol;
        for (int k = map[i].size() - 1; k >= 0; --k) {
            pol.emplace_back(map[i][k]);
        }

        polygons.emplace_back(pol);

        pmap::geom::FPoints tmpPointsCenter;
        pmap::geom::FPolygons tmpTriangles;
        pmap::triangulation::generateTriangularMeshFromPolygon(pol, 4.0, tmpPointsCenter, tmpTriangles);
        totalNumOfPoints+=tmpPointsCenter.size();
        triangles.emplace_back(tmpTriangles);
        pointsCenter.emplace_back(tmpPointsCenter);
    }
    //TODO end of added code


    //TODO modified row
    //std::string filename = argv[1];
    std::string filename = "GeneratedFiles/dataTSP.txt";
    writeDataTSPIntoFile(filename, totalNumOfPoints, polygons.size(), pointsCenter);
    //TODO end of modified row





    std::string output;
    bool outFlag = false;
    std::string mode = "fast";
    double maxTime = DBL_MAX;
    double tourBudget = 0;
    if (argc == 3) {
        outFlag = true;
        output = argv[2];
    } else if (argc == 6) {
        outFlag = true;
        output = argv[2];
        mode = argv[3];
        maxTime = std::stod(argv[4]);
        tourBudget = std::stod(argv[5]);
    }


    // Load GTSP problem instance
    Parser parser;
    parser.parse2dGtspInstance(filename, vertices, edges, sets, edgeMatrix);

    std::cout << "Loaded problem " << filename << std::endl;
    caller->setSets(sets);
    caller->notify();

    auto timeStart = std::chrono::high_resolution_clock::now();

    Tour tour = solver(caller, mode, maxTime, tourBudget);

    //TODO added code
    pmap::draw::MapDrawer md(map);

    md.openPDF("GeneratedFiles/pic.pdf");
    md.drawMap();

    for (auto &tmpTriangles : triangles) {
        md.drawPolygons(tmpTriangles, 1.0, PMAP_DRAW_COL_RED);
    }

    for (auto &tmpPoints : pointsCenter) {
        md.drawPoints(tmpPoints, PMAP_DRAW_COL_BLUE, 15.0);
    }

    for (Edge e : tour.edges) {
        pmap::geom::FPoint p1(e.from.x, e.from.y);
        pmap::geom::FPoint p2(e.to.x, e.to.y);
        md.drawPoint(p1, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawPoint(p2, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawLine(p1, p2, 5, PMAP_DRAW_COL_GREEN, 1.0);
    }
    md.closePDF();
    std::cout << "the end";

    //TODO end of added code

    //TODO added code



    std::vector<pmap::geom::FPoints> pointsCenter1;
    std::vector<pmap::geom::FPolygons> triangles1;
    std::vector<pmap::geom::FPolygon> polygons1;
    int totalNumOfPoints1 = 0;

    for(auto &vertex : tour.vertices){
        int indexPolygon = vertex.setId;
        int indexTriangle = vertex.id;
        for (int i = 0; i < indexPolygon; ++i) {
            indexTriangle -= sets.at(i).vertices.size();
        }
        indexTriangle--;

        pmap::geom::FPolygon pol = triangles.at(indexPolygon).at(indexTriangle);
        polygons1.emplace_back(pol);
        pmap::geom::FPoints tmpPointsCenter;
        pmap::geom::FPolygons tmpTriangles;
        pmap::triangulation::generateTriangularMeshFromPolygon(pol, 1.0f, tmpPointsCenter, tmpTriangles);
        totalNumOfPoints1 += tmpPointsCenter.size();
        triangles1.emplace_back(tmpTriangles);
        pointsCenter1.emplace_back(tmpPointsCenter);
    }



    std::string filename1 = "GeneratedFiles/dataTSP1.txt";
    writeDataTSPIntoFile(filename1, totalNumOfPoints1, polygons1.size(), pointsCenter1);

    ///plan the tour
    vertices.clear();
    edges.clear();
    sets.clear();
    edgeMatrix.clear();
    Parser parser1;
    parser1.parse2dGtspInstance(filename1, vertices, edges, sets, edgeMatrix);
    caller->setSets(sets);
    caller->notify();
    Tour tour1 = solver(caller, mode, maxTime, tourBudget);

    ///draw the pic
    md.openPDF("GeneratedFiles/pic1.pdf");
    md.drawMap();

    for (auto &tmpTriangles : triangles1) {
        md.drawPolygons(tmpTriangles, 1.0, PMAP_DRAW_COL_RED);
    }

    for (auto &tmpPoints : pointsCenter1) {
        md.drawPoints(tmpPoints, PMAP_DRAW_COL_BLUE, 15.0);
    }

    for (Edge e : tour1.edges) {
        pmap::geom::FPoint p1(e.from.x, e.from.y);
        pmap::geom::FPoint p2(e.to.x, e.to.y);
        md.drawPoint(p1, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawPoint(p2, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawLine(p1, p2, 5, PMAP_DRAW_COL_GREEN, 1.0);
    }
    md.closePDF();
    std::cout << "the end";




    //TODO end of added code


    auto t_now = std::chrono::high_resolution_clock::now();
    double time = std::chrono::duration<double, std::milli>(t_now - timeStart).count();

    if (outFlag) {
        std::ofstream outputFile;
        std::ifstream f(output);
        if (!f.good()) {
            outputFile.open (output, std::ofstream::app);
            outputFile << "problem" << " mode" << " maxTime(s)" << " tourBudget" << " time(s)" << " m" << " n"<< " weight\n";
            outputFile << filename << " "<< mode << " " << maxTime << " " << tourBudget << " " << time/1000 << " " << sets.size() << " " << vertices.size() << " " << getTourWeight(tour) << std::endl;
        } else {
            outputFile.open (output, std::ofstream::app);
            outputFile << filename << " "<< mode << " " << maxTime << " " << tourBudget << " " << time/1000 << " " << sets.size() << " " << vertices.size() << " " << getTourWeight(tour) << std::endl;
        }
        outputFile.close();
    }
}



double min(double a, double b) {
    return (a <= b) ? a : b;
}

double max(double a, double b) {
    return (a >= b) ? a : b;
}

void Planner::precomputeSetVertexDistances() {
    for (Vertex u:vertices) {
        for (Set set:sets) {
            if (u.setId != set.id) {
                // dist(set, u) = min{min{w(u, v), w(v, u)}}, where v is from set
                double minDistance = DBL_MAX;
                for (Vertex v:set.vertices) {
                    double dist1 = edgeMatrix[u.id][v.id].weight;
                    double dist2 = edgeMatrix[v.id][u.id].weight;
                    double smaller = min(dist1, dist2);
                    minDistance = min(minDistance, smaller);
                }
                setVertexDistances.insert(std::make_pair(std::make_pair(set.id, u.id), minDistance));
            }
        }
    }
}

void Planner::initHeuristics() {
    // Adding cheapest insertion heuristic
    Heuristic h = Heuristic("cheapest", -1, 0);
    insertionHeuristics.push_back(h);
    // Adding all variants of unified insertion heuristic
    std::vector<double> insertionLambdas = {0, 1.0/2, 1/sqrt(2), 1, sqrt(2), 2, DBL_MAX};
    std::vector<double> mys = {0, 0.25, 0.75};
    for (auto lambda:insertionLambdas) {
        for (auto my:mys) {
            insertionHeuristics.emplace_back("unified", lambda, my);
        }
    }

    // add all variants of unified worst removal and distance removal heuristics
    std::vector<double> removalLambdas = {1/sqrt(2), 1, sqrt(2), 2, DBL_MAX};
    std::vector<std::string> names = {"distance", "worst"};
    for (const auto &name:names) {
        for (auto lambda:removalLambdas) {
            removalHeuristics.emplace_back(name, lambda, -1);
        }
    }
    // add segment removal heuristic
    removalHeuristics.emplace_back("segment", -1, -1);

}



Tour Planner::initRandomTour() {
    std::vector<Set> tmpSets; // we don't want to shuffle sets stored in Planner
    tmpSets = sets;
    Tour tour;
    // uniformly randomly shuffle set indices from 0 to noOfSets - 1
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::shuffle(tmpSets.begin(), tmpSets.end(), generator);

    // uniformly randomly select a vertex from each set
    for (auto set:tmpSets) {
        std::uniform_int_distribution<int> dist(0, set.vertices.size() - 1);
        int randIndex = dist(generator);
        tour.vertices.push_back(set.vertices[randIndex]);
    }

    // get corresponding edges from edgeMatrix
    for (int i = 0; i < tour.vertices.size(); i++) {
        tour.edges.push_back(edgeMatrix[tour.vertices[i].id][tour.vertices[(i + 1) % tour.vertices.size()].id]);
    }

    return tour;
}

/*
 * Chooses a starting vertex v randomly.
 * Remaining vertices are added using the unified insertion heuristic with lambda = 1 and my = 0.75
 */
Tour Planner::initRandomInsertionTour() {
    Tour tour;
    Vertex v1 = getRandomVertex(vertices);
    tour.vertices.push_back(v1);
    Vertex v2 = getRandomVertex(vertices);
    while (v1.setId == v2.setId) {
        v2 = getRandomVertex(vertices);
    }
    tour.vertices.push_back(v2);
    tour.edges.push_back(edgeMatrix[v1.id][v2.id]);
    tour.edges.push_back(edgeMatrix[v2.id][v1.id]);

    double lambda = 1;
    double my = 0.75;

    while(tour.vertices.size() < sets.size()) {
        tour = unifiedInsertion(tour, lambda, my);
    }

    return tour;
}

/*
 * Returns closed partial tour of given length.
 * If length given is too large, returns tour of random length instead.
 * Randomly generated partial tours have length from 1 to sets.size()
 */
Tour Planner::initPartialTour(int length) {
    Tour partialTour;
    std::vector<Set> tmpSets = sets; // we don't want to shuffle sets stored in Planner
    // uniformly randomly shuffle set indices from 0 to noOfSets - 1
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::shuffle(tmpSets.begin(), tmpSets.end(), generator);

    // get a random index from 1 to number of sets - 1
    if (length > tmpSets.size()) {
        std::cout << "Partial tour length given too large (>" << tmpSets.size() << "). Returning randomly long tour."
                  << std::endl;
        std::uniform_int_distribution<int> dist(1, tmpSets.size());
        length = dist(generator);
    }

    // get a subset of tmpSets
    std::vector<Set> tmpSetsSubset(tmpSets.begin(), tmpSets.begin() + length);
    // uniformly randomly select a vertex from each set
    for (auto set:tmpSetsSubset) {
        std::uniform_int_distribution<int> dist(0, set.vertices.size() - 1);
        int randIndex = dist(generator);
        partialTour.vertices.push_back(set.vertices[randIndex]);
    }
    // get corresponding edges from edgeMatrix
    if (partialTour.vertices.size() > 1) {
        for (int i = 0; i < partialTour.vertices.size(); i++) {
            partialTour.edges.push_back(
                    edgeMatrix[partialTour.vertices[i].id][partialTour.vertices[(i + 1) %
                                                                                partialTour.vertices.size()].id]);
        }
    }
    return partialTour;
}



Vertex Planner::getRandomVertex(std::vector<Vertex> vertices) {
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> dist(0, vertices.size() - 1);
    return vertices[dist(generator)];
}

/*
 * Removes vertex v_i and edges (v_i-1, v_i), (v_i, v_i+1)
 * Adds edge (v_i-1.from, v_i+1.to)
 */
Tour Planner::removeVertexFromTour(Tour tour, Vertex vertex) {
    std::vector<Vertex>::iterator vertexIt;
    vertexIt = std::find_if(tour.vertices.begin(), tour.vertices.end(), [&vertex](Vertex const &v) {
        return v.id == vertex.id;
    });
    tour.vertices.erase(vertexIt);
    // remove edges from and to erased vertex
    std::vector<Edge>::iterator edgeIt;
    edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&vertex](Edge const &e) {
        return e.from.id == vertex.id;
    });
    int toId = edgeIt->to.id;
    tour.edges.erase(edgeIt);
    edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&vertex](Edge const &e) {
        return e.to.id == vertex.id;
    });
    int fromId = edgeIt->from.id;
    tour.edges.erase(edgeIt);
    // insert edge connecting disconnected vertices
    tour.edges.push_back(edgeMatrix[fromId][toId]);
    return tour;
}

double Planner::getTourWeight(Tour tour) {
    double weight = 0;
    for (auto e:tour.edges) {
        weight += e.weight;
    }
    return weight;
}



bool compareSetsMinDist(Set set1, Set set2) {
    return (set1.minDist < set2.minDist);
}

/*
 * Performs set selection for nearest, random or farthest insertion
 * lambda = 0 ... nearest insertion
 * lambda = 1 ... random insertion
 * lambda = DBL_MAX ... farthest insertion
 */
Set Planner::unifiedSetSelection(Tour partialTour, double lambda) {
    // get sets, that are not in partialTour (P_V \ P_T)
    std::vector<Set> unusedSets;
    std::vector<Vertex>::iterator it;
    for (int i = 0; i < sets.size(); i++) {
        it = std::find_if(partialTour.vertices.begin(), partialTour.vertices.end(), [&i](Vertex const &v) {
            return v.setId == i;
        });
        if (it == partialTour.vertices.end())
            unusedSets.push_back(sets[i]);
    }
    // for each unused set V_i, define the minimum distance d_i = min dist(V_i, u), where u is from V_T (vertices in partial tour)
    for (auto &set:unusedSets) {
        double minDist = DBL_MAX;
        for (Vertex v:partialTour.vertices) {
            double dist = setVertexDistances[std::make_pair(set.id, v.id)];
            if (dist < minDist) minDist = dist;
        }
        set.minDist = minDist;
    }
    // randomly select k = {0...l - 1} according to the unnormalized probability mass function {lambda^0, lambda^1, ... lambda^(l-1)}
    int l = unusedSets.size();
    std::vector<double> weights(l);
    for (int i = 0; i < l; i++) {
        double weight = std::pow(lambda, i);
        if (weight == std::numeric_limits<double>::infinity()) weight = DBL_MAX;
        weights[i] = weight;
    }
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int k = distribution(generator); // generates random number from 0 to l-1, according to weights given
    // sort unused sets according to minDist, take set at index k
    std::sort(unusedSets.begin(), unusedSets.end(), compareSetsMinDist);
    // Pick the unused set with the k-th smallest minDist to the tour
    return unusedSets[k];
}

Set Planner::cheapestSetSelection(Tour partialTour) {
    // get sets, that are not in partialTour (P_V \ P_T)
    std::vector<Set> unusedSets;
    std::vector<Vertex>::iterator it;
    for (int i = 0; i < sets.size(); i++) {
        it = std::find_if(partialTour.vertices.begin(), partialTour.vertices.end(), [&i](Vertex const &v) {
            return v.setId == i;
        });
        if (it == partialTour.vertices.end())
            unusedSets.push_back(sets[i]);
    }
    // pick the set V_i that contains the vertex v that minimizes the insertion cost w(x,v) + w(v,y) - w(x,y)
    double cost;
    for (auto &set:unusedSets) {
        set.minCost = DBL_MAX;
        for (auto v:set.vertices) {
            for (auto e:partialTour.edges) {
                cost = edgeMatrix[e.from.id][v.id].weight + edgeMatrix[v.id][e.to.id].weight - e.weight;
                if (cost < set.minCost) set.minCost = cost;
            }
        }
    }
    double minCost = DBL_MAX;
    Set minSet;
    for (auto set:unusedSets) {
        if (set.minCost < minCost) {
            minCost = set.minCost;
            minSet = set;
        }
    }
    return minSet;
}

/*
 * Performs one step of unified insertion heuristics.
 * lambda = 0 ... nearest set insertion
 * lambda = 1 ... random set insertion
 * lambda = -1 ... cheapest insertion
 * lambda = DBL_MAX ... farthest set insertion
 * partialTour given must include at least 2 vertices and at most sets.size() - 1 vertices
 */
Tour Planner::unifiedInsertion(Tour partialTour, double lambda, double my) {
    if (partialTour.vertices.size() < 2 || partialTour.edges.empty()) {
        std::cout << "Unified insertion: given tour too short. Returning unchanged tour." << std::endl;
    } else if (partialTour.vertices.size() >= sets.size()) {
        std::cout << "Unified insertion: given tour too long. Returning unchanged tour." << std::endl;
    } else {
        // Pick a set V_i in P_V \ P_T
        Set V_i;
        if (lambda == -1) {
            V_i = cheapestSetSelection(partialTour);
        } else {
            V_i = unifiedSetSelection(partialTour, lambda);
        }
        // Find an edge (x, y) from E_T and vertex v from V_i that minimizes (1 + rand)(w(x,v) + w(v,y) - w(x,y))
        // rand = uniform random number from [0, my]
        double minWeight = DBL_MAX;
        Edge minEdge;
        Vertex minVertex;

        std::default_random_engine generator;
        generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
        std::uniform_real_distribution<double> distribution(0, my);

        for (auto edge:partialTour.edges) {
            Vertex x = edge.from;
            Vertex y = edge.to;
            for (auto v:V_i.vertices) {
                double rand = distribution(generator);
                double weight = (1 + rand)*(edgeMatrix[x.id][v.id].weight + edgeMatrix[v.id][y.id].weight - edge.weight);
                if (weight < minWeight) {
                    minWeight = weight;
                    minEdge = edge;
                    minVertex = v;
                }
            }
        }
        // Delete the edge (x,y) from E_T
        std::vector<Edge>::iterator it;
        it = std::find_if(partialTour.edges.begin(), partialTour.edges.end(), [&minEdge](Edge const &e) {
            return e.id == minEdge.id;
        });
        partialTour.edges.erase(it);
        // add the edges (x,v), (v,y) to E_T
        partialTour.edges.push_back(edgeMatrix[minEdge.from.id][minVertex.id]);
        partialTour.edges.push_back(edgeMatrix[minVertex.id][minEdge.to.id]);
        // Add v to V_T
        partialTour.vertices.push_back(minVertex);
        // return T
    }
    return partialTour;
}



/*
 * Removes a continuous segment of the tour of length N_r.
 */
Tour Planner::segmentRemoval(Tour tour, int N_r) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "segmentRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "segmentRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        // Uniformly randomly select a vertex
        std::default_random_engine generator;
        generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
        std::uniform_int_distribution<int> dist(0, tour.vertices.size() - 1);
        int randIndex = dist(generator);
        Vertex firstVertex = tour.vertices[randIndex];
        // remove currentEdge and nextVertex N_r times
        std::vector<Edge>::iterator edgeIt;
        std::vector<Vertex>::iterator vertexIt;
        Vertex previousVertex = firstVertex;
        Vertex nextVertex;
        Edge currentEdge;
        for (int i = 0; i < N_r; i++) {
            // find edge from previousVertex
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&previousVertex](Edge const &e) {
                return e.from.id == previousVertex.id;
            });
            currentEdge = *edgeIt;
            // find nextVertex
            nextVertex = currentEdge.to;
            vertexIt = std::find_if(tour.vertices.begin(), tour.vertices.end(), [&nextVertex](Vertex const &v) {
                return v.id == nextVertex.id;
            });
            // erase both
            tour.edges.erase(edgeIt);
            tour.vertices.erase(vertexIt);
            previousVertex = nextVertex;
        }
        // erase one remaining edge from the segment
        edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&previousVertex](Edge const &e) {
            return e.from.id == previousVertex.id;
        });
        nextVertex = edgeIt->to;
        tour.edges.erase(edgeIt);
        // make tour closed again
        tour.edges.push_back(edgeMatrix[firstVertex.id][nextVertex.id]);
    }

    return tour;
}

Tour Planner::distanceRemoval(Tour tour, int N_r, double lambda) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "distanceRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "distanceRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        std::vector<Vertex> V_removed;
        // Randomly remove a vertex from T, add it to V_removed
        Vertex vertex = getRandomVertex(tour.vertices);
        tour = removeVertexFromTour(tour, vertex);
        V_removed.push_back(vertex);
        // Perform remaining N_r - 1 removals
        for (int i = 1; i < N_r; i++) {
            // Uniformly randomly select v_seed from V_removed
            Vertex v_seed = getRandomVertex(V_removed);
            // for each v_j from V_T, compute r_j as r_j = min{w(v_seed, v_j), w(v_j, v_seed)}
            for (auto &v:tour.vertices) {
                v.removalCost = min(edgeMatrix[v.id][v_seed.id].weight, edgeMatrix[v_seed.id][v.id].weight);
            }
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

Tour Planner::worstRemoval(Tour tour, int N_r, double lambda) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "worstRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "worstRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        // Calculate removal cost for all vertices
        std::vector<Edge>::iterator edgeIt;
        double prevId, nextId; // vertices ids
        for (auto &v:tour.vertices) {
            v.removalCost = 0;
            // Find edge from v
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&v](Edge const &e) {
                return e.from.id == v.id;
            });
            nextId = edgeIt->to.id;
            v.removalCost += edgeIt->weight;
            // find edge to v
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&v](Edge const &e) {
                return e.to.id == v.id;
            });
            prevId = edgeIt->from.id;
            v.removalCost += edgeIt->weight;
            v.removalCost -= edgeMatrix[prevId][nextId].weight;
        }
        for (int i = 0; i < N_r; i++) {
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

bool compareVerticesRemovalCost(Vertex v1, Vertex v2) {
    return (v1.removalCost < v2.removalCost);
}

Tour Planner::removalFramework(Tour tour, double lambda) {
    // Randomly select k = {0...l - 1} according to the unnormalized probability mass function {lambda^0, lambda^1, ... lambda^(l-1)}
    // Initialize weights
    int l = tour.vertices.size();
    std::vector<double> weights(l);
    for (int i = 0; i < l; i++) {
        double weight = std::pow(lambda, i);
        if (weight == std::numeric_limits<double>::infinity()) weight = DBL_MAX;
        weights[i] = weight;
    }
    // Generate random k
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int k = distribution(generator); // generates random number from 0 to l-1, according to weights given
    // sort vertices according to removalCost, take vertex at index k
    std::sort(tour.vertices.begin(), tour.vertices.end(), compareVerticesRemovalCost);
    // Pick the vertex v_j from V_T  with the kth smallest value r_j
    Vertex v_j = tour.vertices[k];
    // Remove v_j from tour, remove corresponding edges from and to v_j, add edge between disconnected vertices
    tour = removeVertexFromTour(tour, v_j);
    return tour;
}

/*
 * Selects a random number uniformly randomly from range <from, to>
 */
int getRandomNumber(int from, int to) {
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> dist(from, to);
    return dist(generator);
}

Tour Planner::removeInsert(Tour current, std::string phase) {
    // Select a removal heuristic R and insertion heuristic I
    Heuristic * R = selectRemovalHeuristic(phase);
    Heuristic * I = selectInsertionHeuristic(phase);

    // Select the number of vertices to remove, N_r, uniformly randomly from {1,...,N_max}
    unsigned long N_max = sets.size() - 2;
    int N_r = getRandomNumber(1, N_max);

    // Create a copy of current tour
    Tour T_new = current;

    // Remove N_r vertices from T_new using R
    if (R->name == "distance") {
        T_new = distanceRemoval(T_new, N_r, R->lambda);
    } else if (R->name == "worst") {
        T_new = worstRemoval(T_new, N_r, R->lambda);
    } else if (R->name == "segment") {
        T_new = segmentRemoval(T_new, N_r);
    }

    // std::cout << "Removed " << N_r << " vertices, " << R.name << " removal heuristic, lambda = " << R.lambda << std::endl;

    // Insert N_r vertices into T_new using I
    while (T_new.vertices.size() < sets.size()) {
        T_new = unifiedInsertion(T_new, I->lambda, I->my);
    }

    // Update scores for insertion and removal heuristics
    double score = 100 * max(getTourWeight(current) - getTourWeight(T_new), 0)/getTourWeight(current);
    I->scores[phase] += score;
    I->counts[phase] += 1;
    R->scores[phase] += score;
    R->counts[phase] += 1;

    // std::cout << "Inserted " << N_r << " vertices, " << I.name << " insertion heuristic, lambda = " << I.lambda << ", my = " << I.my << std::endl;
    return T_new;
}



/*
 * Returns an insertion heuristic from the insertion heuristics bank.
 * Heuristic is chosen randomly according to a standard roulette wheel mechanism.
 */
Heuristic * Planner::selectInsertionHeuristic(std::string phase) {
    std::vector<double> weights;
    for (auto h:insertionHeuristics) {
        weights.emplace_back(h.weights[phase]);
    }
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator); // generates random number from 0 to l-1, according to weights given
    return &insertionHeuristics[index];
}

/*
 * Returns a removal heuristic from the removal heuristics bank.
 * Heuristic is chosen randomly according to a standard roulette wheel mechanism.
 */
Heuristic * Planner::selectRemovalHeuristic(std::string phase) {
    std::vector<double> weights;
    for (auto h:removalHeuristics) {
        weights.emplace_back(h.weights[phase]);
    }
    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator); // generates random number from 0 to l-1, according to weights given
    return &removalHeuristics[index];
}

void Planner::updateHeuristicsWeights(double epsilon) {
    std::string phasesArray[] = {"early", "mid", "late"};
    std::vector<std::string> phases(phasesArray, phasesArray + sizeof(phasesArray)/ sizeof(std::string));

    for (const auto &phase:phases) {
        for (auto &h:insertionHeuristics) {
            if (h.counts[phase] > 0) {
                h.weights[phase] = epsilon * h.scores[phase]/h.counts[phase] + (1 - epsilon) * h.weights[phase];
            }
            h.scores[phase] = 0;
            h.counts[phase] = 0;
        }
        for (auto &h:removalHeuristics) {
            if (h.counts[phase] > 0) {
                h.weights[phase] = epsilon * h.scores[phase]/h.counts[phase] + (1 - epsilon) * h.weights[phase];
            }
            h.scores[phase] = 0;
            h.counts[phase] = 0;
        }
    }

}

void Planner::printWeights() {
    std::cout << std::endl << "Insertion heuristics" << std::endl;
    for (auto h:insertionHeuristics) {
        std::cout << h.name << " lambda:" << h.lambda << " my:" << h.my << " " << std::endl << "    ";
        for (auto w:h.weights) {
            std::cout << w.first << ": " << w.second << "   ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << "Removal heuristics" << std::endl;
    for (auto h:removalHeuristics) {
        std::cout << h.name << " lambda:" << h.lambda << " my:" << h.my << " " << std::endl << "    ";
        for (auto w:h.weights) {
            std::cout << w.first << ": " << w.second << "   ";
        }
        std::cout << std::endl;
    }

}

bool Planner::acceptTrial(double trialCost, double currentCost, double temperature) {
    double prob1 = exp((currentCost - trialCost)/temperature);
    double prob2 = 1;
    double prob = min(prob1, prob2);

    std::vector<double> weights;
    weights.emplace_back(1-prob); // prob of not accepting at position 0
    weights.emplace_back(prob); // prob of accepting at position 1

    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}

bool Planner::acceptTrialNoParam(double trialCost, double currentCost, double probAccept) {
    if (trialCost < currentCost) return true;

    std::vector<double> weights;
    weights.emplace_back(1-probAccept); // prob of not accepting at position 0
    weights.emplace_back(probAccept); // prob of accepting at position 1

    std::default_random_engine generator;
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}


/*
 * Optimizes the tour given, while keeping the set ordering fixed.
 * Optimization is achieved by performing BFS search.
 */
Tour Planner::reOpt(Tour tour) {
    // find smallest set, to start with
    int minSetId = 0;
    unsigned long minSetSize = LONG_MAX;
    for (auto set:sets) {
        if (set.vertices.size() < minSetSize) {
            minSetSize = set.vertices.size();
            minSetId = set.id;
        }
    }

    // reconstruct sets order from tour.edges
    int firstSetId = minSetId;
    int currentSetId = firstSetId;
    std::vector<int> setsOrdering;
    for (int i = 0; i < sets.size(); i++) {
        for (auto e:tour.edges) {
            if (e.from.setId == currentSetId) {
                setsOrdering.push_back(currentSetId);
                currentSetId = e.to.setId;
                break;
            }
        }
    }

    for (auto &v:sets[firstSetId].vertices) v.BFSWeight = 0;

    Tour bestTour = tour;
    for (auto &vStart:sets[firstSetId].vertices) {
        int nextSetId;
        for (int i = 0; i < sets.size() - 1; i++) {
            if (i == 0) { // fill BFSWeights from start vertex to vertices in first set
                nextSetId = setsOrdering[i+1];
                for (auto &v:sets[nextSetId].vertices) {
                    v.BFSWeight = edgeMatrix[vStart.id][v.id].weight;
                    v.BFSPrevId = vStart.id;
                }
            } else { // fill BFSWeights from all vertices in i-th set to all vertices in i+1-th set
                currentSetId = setsOrdering[i];
                nextSetId = setsOrdering[i+1];
                for (auto &v:sets[nextSetId].vertices) v.BFSWeight = DBL_MAX;
                for (auto &vFrom:sets[currentSetId].vertices) {
                    for (auto &vTo:sets[nextSetId].vertices) {
                        double newWeight = vFrom.BFSWeight + edgeMatrix[vFrom.id][vTo.id].weight;
                        if (newWeight < vTo.BFSWeight) {
                            vTo.BFSWeight = newWeight;
                            vTo.BFSPrevId = vFrom.id;
                        }
                    }
                }
            }
        }
        // Add weight of edge from vertices in last set to first element
        for (auto &v:sets[setsOrdering[sets.size()-1]].vertices) {
            v.BFSWeight += edgeMatrix[v.id][vStart.id].weight;
        }
        // Reconstruct tour from last set to first
        Tour newTour;
        double minWeight = DBL_MAX;
        Vertex minVertex;
        for (auto &v:sets[setsOrdering[sets.size()-1]].vertices) {
            if(v.BFSWeight < minWeight) {
                minWeight = v.BFSWeight;
                minVertex = v;
            }
        }
        newTour.vertices.insert(newTour.vertices.begin(), minVertex);
        Vertex nextVertex = minVertex;
        for (int i = sets.size() - 2; i >= 0; i--) {
            currentSetId = setsOrdering[i];
            Vertex currentVertex;
            for (auto &v:sets[currentSetId].vertices) {
                if (v.id == nextVertex.BFSPrevId) {
                    currentVertex = v;
                }
            }
            newTour.vertices.insert(newTour.vertices.begin(), currentVertex);
            nextVertex = currentVertex;
        }
        // add edges to newTour
        for (int i = 0; i < newTour.vertices.size(); i++) {
            newTour.edges.push_back(edgeMatrix[newTour.vertices[i].id][newTour.vertices[(i + 1) % newTour.vertices.size()].id]);
        }

        if (getTourWeight(newTour) < getTourWeight(bestTour)) bestTour = newTour;
    }

    return bestTour;
}

/*
 * Optimizes the tour given by randomly changing set ordering.
 */
Tour Planner::moveOpt(Tour tour, int NMove) {
    Tour bestTour = tour;
    for (int i = 0; i < NMove; i++) {
        // Randomly select vertex v in tour
        Vertex randV = getRandomVertex(tour.vertices);
        // remove v, remove edges from and to v, add edge (from, to)
        tour = removeVertexFromTour(tour, randV);

        int setId = randV.setId;
        double minWeight = DBL_MAX;
        Edge eToRemove;
        Vertex uMin;
        for (auto u:sets[setId].vertices) {
            for (auto e:tour.edges) {
                double weight = edgeMatrix[e.from.id][u.id].weight + edgeMatrix[u.id][e.to.id].weight - e.weight;
                if (weight < minWeight) {
                    minWeight = weight;
                    eToRemove = e;
                    uMin = u;
                }
            }
        }
        // Remove eToRemove
        std::vector<Edge>::iterator edgeIt;
        edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&eToRemove](Edge const &e) {
            return e.id == eToRemove.id;
        });
        tour.edges.erase(edgeIt);
        // Insert uMin and appropriate edges
        tour.vertices.push_back(uMin);
        tour.edges.push_back(edgeMatrix[eToRemove.from.id][uMin.id]);
        tour.edges.push_back(edgeMatrix[uMin.id][eToRemove.to.id]);

        if (getTourWeight(tour) < getTourWeight(bestTour)) bestTour = tour;
    }

    return bestTour;
}

/*
 * Repeatedly performs moveOpt and reOpt, until there is no improvement.
 */
Tour Planner::optCycle(Tour tour, int NMove) {
    double previousWeight = getTourWeight(tour);
    double newWeight = 0;
    while (newWeight < previousWeight) {
        previousWeight = getTourWeight(tour);
        tour = reOpt(tour);
        tour = moveOpt(tour, NMove);
        newWeight = getTourWeight(tour);
        // std::cout << "previous weight: " << previousWeight << std::endl;
        // std::cout << "new weight     : " << newWeight << std::endl;
    }
    return tour;
}



void Planner::animationDemo(glns::Canvas *caller) {
    // loading GTSP instance
    Parser parser;

    //parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    parser.parse2dGtspInstance("../data/4rat12.gtsp", vertices, edges, sets, edgeMatrix);

    caller->setSets(sets);
    caller->notify();
    sleep(1);

    Tour tour;
    for (int i = 0; i < 10; i++) {
        tour = initRandomTour();
        caller->setTour(tour);
        caller->notify();
        sleep(1);
    }

}

void Planner::partialTourDemo(Canvas *caller) {
    Parser parser;

    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    // parser.parse2dGtspInstance("../data/4rat12.gtsp", vertices, edges, sets, edgeMatrix);

    caller->setSets(sets);
    caller->notify();

    Tour partialTour = initPartialTour(40);
    caller->setTour(partialTour);
    caller->notify();

    /*
    std::cout << partialTour.vertices.size() << std::endl;
    std::cout << partialTour.edges.size() << std::endl;

    for (auto e:partialTour.edges) {
        std::cout << e.id << std::endl;
        std::cout << e.weight << std::endl;
        std::cout << e.from.id << std::endl;
        std::cout << e.to.id << std::endl;
    }
    */

    std::cout << "No of sets: " << sets.size() << std::endl;
    std::cout << "Length of partial tour: " << partialTour.vertices.size() << std::endl;
}

void Planner::unifiedInsertionDemo(Canvas *caller) {
    Parser parser;

    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    // parser.parse2dGtspInstance("../data/4rat12.gtsp", vertices, edges, sets, edgeMatrix);

    caller->setSets(sets);
    caller->notify();

    precomputeSetVertexDistances();
    Tour startingPartialTour = initPartialTour(2);

    std::vector<double> lambdas = {0, 1, 2, DBL_MAX};
    std::vector<std::string> names = {"nearest set", "random set", "cheapest insertion", "farthest set"};
    double my = 0.25;

    for (int i = 0; i < lambdas.size(); i++) {
        Tour partialTour = startingPartialTour;
        caller->setTour(partialTour);
        caller->notify();
        std::cout << "Insertion heuristic: " << names[i] << std::endl;
        sleep(3);

        while (partialTour.vertices.size() < sets.size()) {
            partialTour = unifiedInsertion(partialTour, lambdas[i], my);
            caller->setTour(partialTour);
            caller->notify();
            usleep(500000);
        }

        double cost = 0;
        for (auto e:partialTour.edges) {
            cost += e.weight;
        }
        std::cout << "Cost: " << cost << std::endl;

        sleep(3);
    }
}

void Planner::removalHeuristicsDemo(Canvas *caller) {
    Parser parser;

    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    // parser.parse2dGtspInstance("../data/4rat12.gtsp", vertices, edges, sets, edgeMatrix);

    caller->setSets(sets);
    caller->notify();

    precomputeSetVertexDistances();

    Tour partialTour = initPartialTour(2);

    std::cout << "Initial tour of length 2" << std::endl;
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Constructing complete tour using farthest insertion" << std::endl;
    double lambda = DBL_MAX;
    double my = 0.25;
    while (partialTour.vertices.size() < sets.size()) {
        partialTour = unifiedInsertion(partialTour, lambda, my);
        caller->setTour(partialTour);
        caller->notify();
        usleep(500000);
    }

    /*
    Tour partialTour = initRandomTour();
    caller->setTour(partialTour);
    caller->notify();
    double lambda;
    */

    sleep(5);
    Tour completeTour = partialTour;


    int N_r = 5;

    std::cout << "Removed " << N_r << " vertices using distance removal heuristic" << std::endl;
    lambda = 0;
    partialTour = distanceRemoval(partialTour, N_r, lambda);
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Original tour reset" << std::endl;
    partialTour = completeTour;
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Removed " << N_r << " vertices using worst removal heuristic" << std::endl;
    lambda = DBL_MAX;
    partialTour = worstRemoval(partialTour, N_r, lambda);
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Original tour reset" << std::endl;
    partialTour = completeTour;
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Removed " << N_r << " vertices using segment removal heuristic" << std::endl;
    partialTour = segmentRemoval(partialTour, N_r);
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Original tour reset" << std::endl;
    partialTour = completeTour;
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Removed " << N_r << " vertices using random removal" << std::endl;
    lambda = 1;
    partialTour = worstRemoval(partialTour, N_r, lambda);
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);

    std::cout << "Original tour reset" << std::endl;
    partialTour = completeTour;
    caller->setTour(partialTour);
    caller->notify();
    sleep(5);
}

void Planner::givenTourDisplayDemo(Canvas *caller) {
    // Load GTSP problem instance
    Parser parser;
    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    caller->setSets(sets);
    caller->notify();

    Tour tour;
    std::vector<int> vIds{30, 45, 56, 54, 67, 82, 83, 95, 107, 120, 133, 158, 171, 174, 175, 190, 193, 194, 180, 152, 154, 141, 128, 114, 116, 102, 88, 76, 64, 63, 50, 37, 36, 24, 8, 4, 3, 15, 16};
    for (int i = 0; i < vIds.size(); i++) {
        for (auto v:vertices) {
            if (v.id == vIds[i]) {
                tour.vertices.push_back(v);
            }
        }
        tour.edges.push_back(edgeMatrix[vIds[i]][vIds[(i+1) % vIds.size()]]);
    }

    std::cout << "cost: " << getTourWeight(tour) << std::endl;
    caller->setTour(tour);
    caller->notify();
}

void Planner::reOptDemo(Canvas *caller) {
    // Load GTSP problem instance
    Parser parser;
    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    initHeuristics();
    Tour tour = initRandomInsertionTour();

    caller->setSets(sets);
    caller->setTour(tour);
    caller->notify();

    Tour newTour = reOpt(tour);
    caller->setBestTour(newTour);
    caller->notify();

    std::cout << "Order of sets is fixed, vertices used are optimized using BFS" << std::endl;
    std::cout << "original tour weight: " << getTourWeight(tour) << " (RED)"<< std::endl;
    std::cout << "reOpt optimized tour weight: " << getTourWeight(newTour)<< " (BLACK)" << std::endl;
}

void Planner::moveOptDemo(Canvas *caller) {
    // Load GTSP problem instance
    Parser parser;
    parser.parse2dGtspInstance("../data/39rat195.gtsp", vertices, edges, sets, edgeMatrix);
    initHeuristics();
    Tour tour = initRandomTour();

    caller->setSets(sets);
    caller->setBestTour(tour);
    caller->notify();
    sleep(3);
    std::cout << "optimizing using moveOpt. Initial weight: " << getTourWeight(tour) << std::endl;

    for (int i = 0; i < 1000; i++) {
        tour = moveOpt(tour, 1);
        caller->setBestTour(tour);
        caller->notify();
        std::cout << i << ": " << getTourWeight(tour) << std::endl;
        usleep(100000);
    }
}



Tour Planner::solver(Canvas *caller, std::string mode, double maxTime, double tourBudget) {
    std::cout << "Planning..." << std::endl;
    // Common parameters
    auto numSets = (int) sets.size();
    double acceptPercentage = 0.05;
    double epsilon = 0.5;
    __useconds_t uDelay = 0; // delay after finding a better tour in warm trial; in useconds
    // Mode-specific parameters
    int coldTrials;
    int warmTrials;
    int numIterations;
    // if best tour wasn't improved for latestImprovement consecutive iterations, leave initial descent (mid phase)
    int latestImprovement;
    // if best tour wasn't improved for firstImprovement consecutive iterations and there was not an initial improvement, leave warm restart (late phase)
    int firstImprovement;
    double probAccept;
    int NMax;
    int NMove;
    if (mode == "fast") {
        coldTrials = 3;
        warmTrials = 2;
        numIterations = 60 * numSets;
        latestImprovement = numIterations / 4;
        firstImprovement = numIterations / 6;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(min(20, 0.1 * numSets));
        NMove = NMax;
    } else if (mode == "default") {
        coldTrials = 5;
        warmTrials = 3;
        numIterations = 60 * numSets;
        latestImprovement = numIterations / 2;
        firstImprovement = numIterations / 4;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(min(100, 0.3 * numSets));
        NMove = NMax;
    } else if (mode == "slow") {
        coldTrials = 10;
        warmTrials = 5;
        numIterations = 150 * numSets;
        latestImprovement = numIterations / 3;
        firstImprovement = numIterations / 6;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(0.4 * numSets);
        NMove = NMax;
    } else {
        std::cout << "Invalid mode: " << mode << std::endl;
        return Tour();
    }

    // Init planner counters, flags, variables
    int latestImprovementCnt = 1;
    bool firstImprovementFlag = false;
    int warmTrialsCnt = 0;
    int coldTrialsCnt = 1;
    int totalIterCnt = 0;

    auto timeStart = std::chrono::high_resolution_clock::now();
    Tour lowestT; // best tour found overall

    // Precompute all necessary stuff
    precomputeSetVertexDistances();
    
    // Cold trials loop
    while (coldTrialsCnt <= coldTrials) {
        // build tour from scratch on a cold restart
        Tour bestT = initRandomInsertionTour(); // best tour in this trial
        if (lowestT.vertices.empty()) {
            lowestT = bestT;
        } else {
            if (getTourWeight(lowestT) > getTourWeight(bestT)) lowestT = bestT;
        }

        std::string phase = "early";

        // Update selection weights
        if (coldTrialsCnt == 1) {
            initHeuristics();
        } else {
            updateHeuristicsWeights(epsilon);
            // printWeights();
        }

        // Warm restarts loop
        while (warmTrialsCnt <= warmTrials) {
            int iterCount = 1;
            Tour currentT = bestT;
            double temperature = 1.442 * acceptPercentage * getTourWeight(bestT);
            double cooling_rate = pow(((0.0005 * getTourWeight(lowestT))/(acceptPercentage * getTourWeight(currentT))), 1.0/numIterations);

            // If warm restart, use lower temperature
            if (warmTrialsCnt > 0) {
                temperature *= pow(cooling_rate, numIterations/2.0);
                phase = "late";
            }

            //
            while(latestImprovementCnt <= (firstImprovementFlag ? latestImprovement : firstImprovement)) {
                // Move to mid phase after half iterations
                if ((iterCount > numIterations/2) && (phase == "early")) {
                    phase = "mid";
                }

                Tour trial = removeInsert(currentT, phase);

                // Decide whether or not to accept trial
                if (acceptTrialNoParam(getTourWeight(trial), getTourWeight(currentT), probAccept) || acceptTrial(getTourWeight(trial), getTourWeight(currentT), temperature)) {
                    if (mode == "slow") trial = optCycle(trial, NMove);
                    currentT = trial;
                }



                if(getTourWeight(currentT) < getTourWeight(bestT)) {
                    latestImprovementCnt = 1;
                    firstImprovementFlag = true;
                    if ((coldTrialsCnt > 1) && (warmTrialsCnt > 1)) {
                        warmTrialsCnt++;
                    }
                    currentT = optCycle(currentT, NMove); // Locally reoptimize current tour

                    bestT = currentT;
                    caller->setTour(bestT);
                    caller->notify();
                    usleep(uDelay);
                } else {
                    latestImprovementCnt++;
                }

                // time limit and budget limit check
                auto t_now = std::chrono::high_resolution_clock::now();
                double timeFromStart = std::chrono::duration<double, std::milli>(t_now - timeStart).count();
                if ((timeFromStart/1000 > maxTime) || (getTourWeight(bestT) < tourBudget)) {
                    if (timeFromStart/1000 > maxTime) std::cout << "Max time exceeded" << std::endl;
                    if (getTourWeight(bestT) < tourBudget) std::cout << "Tour better than budget found" << std::endl;
                    if (getTourWeight(lowestT) > getTourWeight(bestT)) lowestT = bestT;
                    std::cout << "lowest weight: " << getTourWeight(lowestT) << std::endl;
                    caller->setBestTour(lowestT);
                    Tour emptyTour;
                    caller->setTour(emptyTour);
                    caller->notify();
                    return lowestT;
                }

//                std::cout << "timeFromStart: " << timeFromStart << " ms  coldTrialsCnt: " << coldTrialsCnt << "    warmTrialsCnt: " << warmTrialsCnt << "  phase: " << phase << "  iterCount: " << iterCount << "     temperature: " << temperature << " best weight: " << getTourWeight(bestT) << std::endl;

                // cool the temperature
                temperature *=cooling_rate;
                iterCount++;
                totalIterCnt++;

            }

            warmTrialsCnt++;
            latestImprovementCnt = 1;
            firstImprovementFlag = false;
        } // Warm trials loop

        if (getTourWeight(lowestT) > getTourWeight(bestT)){
            lowestT = bestT;
            caller->setBestTour(lowestT);
            caller->notify();
        }
        warmTrialsCnt = 0;
        coldTrialsCnt++;
    } // Cold trials loop

    std::cout << "Best tour found: ";
    for (auto v:lowestT.vertices) std::cout << v.id << " ";
    std::cout << std::endl;
    std::cout << "Weight: " << getTourWeight(lowestT) << std::endl;
    Tour emptyTour;
    caller->setTour(emptyTour);
    caller->notify();
    // for (auto v:lowest.vertices) std::cout << v.id << " ";
    // std::cout << std::endl;

    return lowestT;
}
































