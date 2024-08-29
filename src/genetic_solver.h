#pragma once

#include "instance.h"
#include "route.h"


struct GeneticSolution {
    // List of routes in the solution
    std::vector<Route> routes;
    // Map from vehicle id to the index of the route in the routes vector
    std::map<int, int> vehicle_to_route;
    // Objective value of the solution
    double objective;
};

/*
    @brief Evaluates a solution by summing the costs of all the routes
*/
double evaluate(const GeneticSolution& solution, const Instance& instance);


/*
    @brief Repairs a solution that results from a crossover by removing duplicate interventions in the most advantageous way
*/
void repair(GeneticSolution& solution, const Instance& instance);


/*
    @brief Generates a new solution from two parents by picking routes from each parent at random
*/
GeneticSolution crossover(const GeneticSolution& parent1, const GeneticSolution& parent2, const Instance& instance);


/*
    @brief Initializes a new GeneticSolution using the greedy heuristic
*/
GeneticSolution greedy_initializer(const Instance& instance);


/*
    @brief Performs the genetic algorithm to solve the technician routing problem
*/
void genetic_algorithm(const Instance& instance);
