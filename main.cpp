#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_set>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <random>
#include <limits>
#include <iostream>
#include <math.h>

#define MAX_PATH_LEN 720
#define PROBLEMS_DIR "Training Problems"

typedef struct {
    double_t x;
    double_t y;
} Point;

typedef struct {
    Point pickup;
    Point dropoff;
    size_t loadNumber;
} Load;

typedef struct {
    std::vector<double_t> edges;
    double_t weight;
    size_t index;
} Node;

typedef std::vector<Node> Graph;

typedef std::vector<Node> Route;

typedef struct {
    std::vector<Route> routes;
    std::vector<double_t> lengths;
    std::unordered_set<size_t> unvisited;
} Solution;

double_t solutionCost(std::vector<double_t>& lengths, double_t fixedPathCost) {
    return lengths.size() * fixedPathCost 
        + std::accumulate(lengths.begin(), lengths.end(), 0);
}


/*
 * @brief Creates a graph of the loads
 * @returns A graph of the loads
*/
Graph makeGraph(const std::vector<Load>& loads) {
    std::vector<Node> graph;
    for (auto load : loads) {
        std::vector<double_t> edges;
        for (auto load2 : loads) {
            if (load.loadNumber == load2.loadNumber) {
                edges.push_back(0);
            } else {
                // go from end of first load to start of second load
                edges.push_back(sqrt(pow(load.dropoff.x - load2.pickup.x, 2) + pow(load.dropoff.y - load2.pickup.y, 2)));
            }
        }
        graph.push_back(Node{
            .weight = sqrt(pow(load.pickup.x - load.dropoff.x, 2) + pow(load.pickup.y - load.dropoff.y, 2)),
            .edges = edges,
            .index = graph.size()
        });
    }
    return graph;
}


/**
 * @brief Finds the nearest neighbor solution to the problem
 * described by the graph with the given max path length constraint.
*/
Solution nearestNeighborSolution(Graph& graph, double_t maxPathLength) {
    std::vector<Route> routes;
    std::vector<double_t> lengths;

    std::unordered_set<size_t> unvisited;
    for (size_t i = 1; i < graph.size(); i++) {
        unvisited.insert(i);
    }

    while(unvisited.size() > 0) {
        Route path;
        path.push_back(graph[0]);
        double_t pathLength = 0;

        while(unvisited.size() > 0) {
            Node currentNode = path.back();

            int nearest = -1;
            for(int i = 1; i < currentNode.edges.size(); i++) {
                // if the node is the current node or it has already been visited, skip it
                if(i == currentNode.index || unvisited.find(i) == unvisited.end()) continue;
                if(nearest == -1 || currentNode.edges[i] < currentNode.edges[nearest]) {
                    nearest = i;
                }
            }

            bool pathTooLong = pathLength 
                + currentNode.edges[nearest]
                + graph[nearest].weight
                + graph[nearest].edges[0]
                > maxPathLength;
            bool depotIsCloser = currentNode.index != 0 
                && currentNode.edges[0] < currentNode.edges[nearest];
            if(pathTooLong || depotIsCloser) {
                break;
            }
            path.push_back(graph[nearest]);
            pathLength += currentNode.edges[nearest];
            unvisited.erase(nearest);
        }
        path.push_back(graph[0]);
        pathLength += path.back().edges[0];
        routes.push_back(path);
        lengths.push_back(pathLength);
    }
    return Solution{
        .routes = routes,
        .lengths = lengths,
        .unvisited = unvisited // empty
    };
}


/**
 * Reads a vehicle routing problem problem from the given 
 * file path and returns a solution to the problem.
*/
Solution solveProblem(std::string problemPath) {
    std::cout << "Reading problem from " << problemPath << std::endl;

    // 0,0 is the depot
    std::vector<Load> loads {
        Load{
            .pickup = Point{0,0},
            .dropoff = Point{0,0},
            .loadNumber = 0
        }
    };

    // Read loads into vector
    std::ifstream problem(problemPath);
    std::string line;
    getline(problem, line); // column headers
    size_t space1;
    size_t space2;
    size_t pickupComma;
    size_t dropoffComma;
    std::string pickup;
    std::string dropoff;
    while (getline(problem, line)) {
        space1 = line.find('(');
        space2 = line.find('(', space1+1);

        pickup = line.substr(space1 + 1, space2 - space1 - 3);
        pickupComma = pickup.find(',');
        const double_t pickupX = std::stod(pickup.substr(0, pickupComma));
        const double_t pickupY = std::stod(pickup.substr(pickupComma + 1));

        dropoff = line.substr(space2 + 1, line.length() - space2 - 2);
        dropoffComma = dropoff.find(',');
        const double_t dropoffX = std::stod(dropoff.substr(0, dropoffComma));
        const double_t dropoffY = std::stod(dropoff.substr(dropoffComma + 1));
        loads.push_back(
            Load{
                .pickup = Point{pickupX, pickupY}, 
                .dropoff = Point{dropoffX, dropoffY},
                .loadNumber = loads.size()
            }
        );
    }
    problem.close();

    auto graph = makeGraph(loads);
    auto solution = nearestNeighborSolution(graph, MAX_PATH_LEN);

    return solution;
}


/**
 * Solves problems specified by files in PROBLEMS_DIR directory and outputs 
 * the average time to solve the problems and the average cost of the solutions.
*/
int main() {
    double_t totalTime = 0;
    double_t totalCost = 0;
    size_t numProblems = 0;
    for (const auto& dirEntry : std::filesystem::recursive_directory_iterator(PROBLEMS_DIR)) {
        auto start = std::chrono::system_clock::now();

        auto plan = solveProblem(dirEntry.path());
        auto cost = solutionCost(plan.lengths, 500);
        totalCost += cost;

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double_t> elapsed_seconds = end-start;
        totalTime += elapsed_seconds.count();
        numProblems++;
    }

    std::cout << "Average time to solve problem: " << totalTime / numProblems << "s\n";
    std::cout << "Average cost: " << totalCost / numProblems << "\n";

    return 0;
}