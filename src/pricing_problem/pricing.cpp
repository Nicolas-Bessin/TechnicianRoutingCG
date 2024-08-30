#include "pricing.h"

#include "pricing_problem/subproblem.h"
#include "routes/route_optimizer.h"

#include <vector>
#include <execution>
#include <memory>


std::vector<Route> solve_pricing_problems_basic(
    const MasterSolution & solution,
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