#include "pricing.h"
#include <map>
#include <iostream>
#include "../pathwyse/core/data/problem.h"

// First, we want to define the network on which we will solve the problem
void find_best_route(const Instance& instance, const Vehicle& vehicle, const vector<double> &alphas, const double beta) {
    // Create a new instance of the Problem class
    Problem problem = Problem();
    // Set the name of the problem
    problem.setName("Constrained Vehicle Routing Problem");
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    // +1 for the "depature" warehouse and +1 for the "arrival" warehouse (even tough it is the same place)
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v + 1;
    int destination = origin + 1;
    // Initialize the problem with the number of nodes
    problem.initProblem(n_interventions_v + 2, true, false);
    problem.setOrigin(origin);
    problem.setDestination(destination);
    problem.printProblem();
    // We use a default cost for the objective function
    double init_value = beta + vehicle.cost;
    //DefaultCost objective = DefaultCost(init_value);
    for (int i = 0; i < n_interventions_v; i++) {
        for (int j = 0; j < n_interventions_v; j++) {
            if (i != j) {
                int next_duration = vehicle.interventions.at(j)->duration;
                // Get the distance between the two interventions
                double distance = metric(vehicle.interventions[i], vehicle.interventions[j], instance.distance_matrix);
                double arc_cost = alphas.at(i) - instance.M * next_duration + instance.cost_per_km * distance;
            }
        }
    }
    return;

}