#include "pricing.h"

#include "pricing_problem/subproblem.h"
#include "routes/route_optimizer.h"

#include <vector>
#include <execution>
#include <memory>
#include <random>
#include <iostream>


std::vector<Route> solve_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    double reduced_cost_threshold
    ){
    using std::vector;
    using std::unique_ptr;

    // Initialize the vehicle order
    // Explore all the vehicles in order at each iteration
    // We formulate it this way to allow for other orders of exploration 
    // Or not exploring all vehicles at each iteration in the future
    vector<int> vehicle_order(instance.vehicles.size());
    std::iota(vehicle_order.begin(), vehicle_order.end(), 0);

    // Initialize the new routes
    vector<Route> new_routes = {};

    double max_reduced_cost = 0;

    // Parallelize the pricing sub problems
    std::for_each(
        std::execution::par_unseq,
        vehicle_order.begin(),
        vehicle_order.end(),
        [&](int v){
            const Vehicle& vehicle = instance.vehicles.at(v);
            unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, using_cyclic_pricing);
            update_pricing_instance(pricing_problem, solution, instance, vehicle);
            Route new_route = solve_pricing_problem(pricing_problem, instance, vehicle, n_ressources_dominance);
            max_reduced_cost = std::max(max_reduced_cost, new_route.reduced_cost);
            if (new_route.reduced_cost> reduced_cost_threshold){
                new_routes.push_back(new_route);              
            }
        }
    );

    return new_routes;
}

std::vector<int> random_cycle(int n, int seed){
    using std::vector;
    using std::rand;
    
    std::random_device rd;
    std::mt19937 gen(seed);


    vector<int> cycle(n);
    // Elements to visit
    vector<int> available(n);
    std::iota(available.begin(), available.end(), 0);
    // Randomly select the first element
    int current = std::uniform_int_distribution<int>(0, n-1)(gen);
    int intial = current;
    available.erase(available.begin() + current);
    // Loop until the set of available elements is empty
    while (!available.empty()){
        // Select the next element
        int next_id = std::uniform_int_distribution<int>(0, available.size()-1)(gen);
        cycle[current] = available[next_id];
        current = available[next_id];
        available.erase(available.begin() + next_id);
    }
    cycle[current] = intial;
    
    return cycle;
}

   
std::vector<Route> solve_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    double reduced_cost_threshold,
    int seed
) {
    using std::vector;
    using std::unique_ptr;

    // First step in generating a random cycle permutation of the vehicles
    vector<int> permutation = random_cycle(instance.vehicles.size(), seed);

    // Initialize the new routes
    vector<Route> new_routes = {};

    // Do the diversifying generation beginning from every vehicle
    for (int initital_vehicle = 0; initital_vehicle < instance.vehicles.size(); initital_vehicle++){
        // Initially, no intervention is covered
        vector<int> covered_interventions = vector<int>(instance.nodes.size(), 0);
        int current_vehicle = initital_vehicle;
        // Go through each vehicle in order of the permutation
        for (int k = 0; k < instance.vehicles.size(); k++){
            const Vehicle& vehicle = vehicle_mask(instance.vehicles.at(current_vehicle), covered_interventions, KEEP_NON_COVERED);
            // If the vehicle has no interventions to cover, we skip it
            if (vehicle.interventions.size() == 0){
                current_vehicle = permutation[current_vehicle];
                continue;
            }
            // Solve the pricing problem for the vehicle
            unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, using_cyclic_pricing);
            update_pricing_instance(pricing_problem, solution, instance, vehicle);
            Route new_route = solve_pricing_problem(pricing_problem, instance, vehicle, n_ressources_dominance);
            // If the reduced cost is greater than the threshold, we add the route
            if (new_route.reduced_cost > reduced_cost_threshold){
                new_routes.push_back(new_route);
                // Update the covered interventions (interventions only, not warehouses)
                for (int i = 1; i < new_route.id_sequence.size() - 1; i++){
                    covered_interventions[new_route.id_sequence[i]] = 1;
                }
            }
            // Update the current vehicle
            current_vehicle = permutation[current_vehicle];
        }
    }

    return new_routes;
}