#include "heuristics.h"

#include <memory>
#include <random>


std::vector<Route> greedy_heuristic(
    const Instance& instance, 
    std::vector<int> vehicle_order, 
    const std::set<std::tuple<int, int, int>>& forbidden_edges,
    const std::set<std::tuple<int, int, int>>& required_edges
    ) {
    using std::vector;
    using std::unique_ptr;
    using std::cout, std::endl;

    vector<Route> routes;
    int n_interventions = instance.number_interventions;
    int n_vehicles = instance.vehicles.size();
    vector<int> covered(n_interventions, 0);

    vector<int> order = vehicle_order;
    // If no order is given, order the vehicles by increasing number of interventions available
    if (order.size() == 0) {
        order.resize(n_vehicles);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&instance](int i, int j) {
            return instance.vehicles[i].interventions.size() < instance.vehicles[j].interventions.size();
        });
    }

    for (const int v_id: order) {
        // Find the vehicle
        const Vehicle& vehicle = instance.vehicles[v_id];
        // Transform the current vehicle to only consider the interventions that are not yet covered
        const Vehicle& v = vehicle_mask(vehicle, covered);
        // Create a pricing problem for the vehicle
        unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, v, forbidden_edges, required_edges);
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


std::vector<Route> greedy_heuristic_duals(
    const Instance& instance, 
    const MasterSolution& master_solution,
    std::vector<int> vehicle_order,
    const std::set<std::tuple<int, int, int>>& forbidden_edges,
    const std::set<std::tuple<int, int, int>>& required_edges
    ) {
    using std::vector;
    using std::unique_ptr;
    using std::cout, std::endl;

    vector<Route> routes;
    int n_interventions = instance.number_interventions;
    int n_vehicles = instance.vehicles.size();
    vector<int> covered(n_interventions, 0);

    vector<int> order = vehicle_order;
    // If no order is given, order the vehicles by increasing number of interventions available
    if (order.size() == 0) {
        order.resize(n_vehicles);
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&instance](int i, int j) {
            return instance.vehicles[i].interventions.size() < instance.vehicles[j].interventions.size();
        });
    }

    for (const int v_id: order) {
        // Find the vehicle
        const Vehicle& vehicle = instance.vehicles[v_id];
        // Transform the current vehicle to only consider the interventions that are not yet covered
        const Vehicle& v = vehicle_mask(vehicle, covered);
        // Create a pricing problem for the vehicle
        unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, v, forbidden_edges, required_edges);
        update_pricing_instance(pricing_problem, master_solution, instance, v);
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