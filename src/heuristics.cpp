#include "heuristics.h"

#include <memory>
#include <random>


std::vector<Route> greedy_heuristic(const Instance& instance) {
    using std::vector;
    using std::unique_ptr;
    using std::cout, std::endl;

    vector<Route> routes;
    int n_interventions = instance.number_interventions;
    int n_vehicles = instance.vehicles.size();
    vector<int> covered(n_interventions, 0);
    
    // Randomly shuffle the vehicles
    std::random_device rd;
    std::mt19937 g(rd());
    vector<int> vehicle_indices(n_vehicles);
    // Fill the vector with the indices of the vehicles
    std::iota(vehicle_indices.begin(), vehicle_indices.end(), 0);
    // Shuffle the indices
    std::shuffle(vehicle_indices.begin(), vehicle_indices.end(), g);
    
    // Enumerate through all vehicles : generate a route that maximizes its benefit
    // Among the interventions that are not yet covered
    for (const int v_id: vehicle_indices) {
        // Find the vehicle
        const Vehicle& vehicle = instance.vehicles[v_id];
        // Transform the current vehicle to only consider the interventions that are not yet covered
        const Vehicle& v = vehicle_mask(vehicle, covered);
        // Create a pricing problem for the vehicle
        unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, v);
        // Update it with zeros for alphas and betas
        update_pricing_instance(pricing_problem, vector<double>(n_interventions, 0), 0, instance, v);
        // Solve the pricing problem
        vector<Route> new_routes = solve_pricing_problem(pricing_problem, 1, instance, v);
        // If no route was found, continue to the next vehicle
        if (new_routes.size() == 0) {
            continue;
        }
        // Only keep the first route
        Route new_route = new_routes[0];
        // Update the covered interventions
        for (int i = 0; i < n_interventions; i++) {
            if (new_route.is_in_route[i] == 1) {
                covered[i] = 1;
            }
        }
        // Add the new route to the list of routes
        routes.push_back(new_route);
    }
    // Print the number of interventions covered
    int n_covered = 0;
    for (int i = 0; i < n_interventions; i++) {
        n_covered += covered[i];
    }
    cout << "Greedy heuristic covered " << n_covered << " interventions" << endl;

    return routes;
}