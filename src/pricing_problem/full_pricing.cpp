#include "full_pricing.h"

#include "pricing_problem/subproblem.h"
#include "routes/route.h"
#include "routes/route_optimizer.h"
#include "clustering/clustering.h"

#include <vector>
#include <execution>
#include <memory>
#include <random>
#include <iostream>
#include <thread>
#include <future>


std::vector<Route> full_pricing_problems_basic(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    SolverMode solver_objective_mode,
    bool using_cyclic_pricing,
    int n_ressources_dominance
    ){
    using std::vector;
    using std::packaged_task;
    using std::future;
    
    auto single_pricer = [&](int v){
            return solve_pricing_problem(instance, instance.vehicles.at(v), solution,
                solver_objective_mode, using_cyclic_pricing, n_ressources_dominance);
        };

    vector<packaged_task<Route(int)>> tasks;
    for (int v : vehicle_order){
        tasks.push_back(packaged_task<Route(int)>(single_pricer));
    }
    vector<future<Route>> futures;
    for (auto& task : tasks){
        futures.push_back(task.get_future());
    }
    // Multi-threaded execution
    vector<std::thread> threads(tasks.size());

    // Sequential execution
    vector<Route> new_routes_parallel(vehicle_order.size());
    for (int i = 0; i < tasks.size(); i++){
        threads[i] = std::thread(std::move(tasks[i]), vehicle_order[i]);
    }

    for (int i = 0; i < tasks.size(); i++){
        threads[i].join();
        new_routes_parallel[i] = futures[i].get();
    }

    return new_routes_parallel;
}

   
std::vector<Route> full_pricing_problems_diversification(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    SolverMode solver_objective_mode,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed
) {
    using std::vector;
    using std::packaged_task;
    using std::future;

    // Randomize the seed
    if (seed == RANDOM_SEED){
        seed = std::chrono::system_clock::now().time_since_epoch().count();
    }

    // First step in generating a random permutation of the vehicle order
    // We do this by shuffling the indexes of the vehicles
    vector<int> permutation(vehicle_order.size());
    std::iota(permutation.begin(), permutation.end(), 0);
    std::shuffle(permutation.begin(), permutation.end(), std::mt19937(seed));

    auto single_diversifier = [&](int initital_vehicle_index){
        vector<Route> private_new_routes = {};
        // Initially, no intervention is covered
        vector<int> covered_interventions = vector<int>(instance.nodes.size(), 0);
        // Go through each vehicle in order of the permutation
        int explore = std::min((int)vehicle_order.size(), 10);
        for (int k = 0; k < explore; k++){
            int current_vehicle = vehicle_order[permutation[(initital_vehicle_index + k) % vehicle_order.size()]];
            const Vehicle& vehicle = vehicle_mask(instance.vehicles.at(current_vehicle), covered_interventions, KEEP_NON_COVERED);
            // If the vehicle has no interventions to cover, we skip it
            if (vehicle.interventions.size() == 0){
                continue;
            }
            // Solve the pricing problem for the vehicle
            Route new_route = solve_pricing_problem(instance, vehicle, solution,
                solver_objective_mode, using_cyclic_pricing, n_ressources_dominance);
            // If the reduced cost is greater than the threshold, we add the route
            if (new_route.id_sequence.size() > 2){
                private_new_routes.push_back(new_route);
                // Update the covered interventions (interventions only, not warehouses)
                for (int i = 1; i < new_route.id_sequence.size() - 1; i++){
                    covered_interventions[new_route.id_sequence[i]] = 1;
                }
            }
        }
        return private_new_routes;
    };


    // We create the tasks
    vector<std::packaged_task<vector<Route>(int)>> tasks;
    for (int i = 0; i < vehicle_order.size(); i++){
        tasks.push_back(packaged_task<vector<Route>(int)>(single_diversifier));
    }
    vector<future<vector<Route>>> futures;
    for (auto& task : tasks){
        futures.push_back(task.get_future());
    }
    // Multi-threaded execution
    vector<std::thread> threads(tasks.size());
    vector<vector<Route>> new_routes_parallel(vehicle_order.size());
    for (int i = 0; i < tasks.size(); i++){
        threads[i] = std::thread(std::move(tasks[i]), i);
    }
    // Get the results
    for (int i = 0; i < tasks.size(); i++){
        threads[i].join();
        new_routes_parallel[i] = futures[i].get();
    }
    // Build the final list of routes
    vector<Route> new_routes;
    for (auto routes : new_routes_parallel){
        if (routes.size() == 0){
            continue;
        }
        new_routes.insert(new_routes.end(), routes.begin(), routes.end());
    }
    return new_routes;
}


std::vector<Route> full_pricing_problems_basic_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    SolverMode solver_objective_mode,
    int delta,
    int pool_size,
    bool verbose
){
    using std::vector;
    using std::packaged_task;
    using std::future;
    
    auto single_pricer = [&](int v){
            return solve_pricing_problem_pulse(instance, instance.vehicles.at(v), solution,
                solver_objective_mode, delta, pool_size, verbose);
        };

    vector<packaged_task<vector<Route>(int)>> tasks;
    for (int v : vehicle_order){
        tasks.push_back(packaged_task<vector<Route>(int)>(single_pricer));
    }
    vector<future<vector<Route>>> futures;
    for (auto& task : tasks){
        futures.push_back(task.get_future());
    }
    // Multi-threaded execution
    vector<std::thread> threads(tasks.size());

    // Sequential execution
    vector<vector<Route>> new_routes_parallel(vehicle_order.size());
    for (int i = 0; i < tasks.size(); i++){
        threads[i] = std::thread(std::move(tasks[i]), vehicle_order[i]);
        //threads[i].join(); // Non-parallel execution
    }

    for (int i = 0; i < tasks.size(); i++){
        threads[i].join();
        new_routes_parallel[i] = futures[i].get();
    }

    // Regroup all the routes in a single vector
    vector<Route> new_routes;
    for (auto routes : new_routes_parallel){
        new_routes.insert(new_routes.end(), routes.begin(), routes.end());
    }

    return new_routes;
}


std::vector<Route> full_pricing_problems_grouped_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::map<int, std::vector<int>> & vehicle_groups,
    SolverMode solver_objective_mode,
    int delta,
    int pool_size,
    bool verbose
) {
    using std::vector;
    using std::packaged_task;
    using std::future;

    auto single_pricer = [&](int id){
            return solve_pricing_problem_pulse_grouped(instance, vehicle_groups.at(id), solution,
                solver_objective_mode, delta, pool_size, verbose);
        };

    vector<packaged_task<vector<Route>(int)>> tasks;
    vector<int> task_id_to_depot = {};
    for (const auto& [id, vehicles] : vehicle_groups){
        tasks.push_back(packaged_task<vector<Route>(int)>(single_pricer));
        task_id_to_depot.push_back(id);
    }
    vector<future<vector<Route>>> futures;
    for (auto& task : tasks){
        futures.push_back(task.get_future());
    }
    // Multi-threaded execution
    vector<std::thread> threads(tasks.size());
    for (int i = 0; i < tasks.size(); i++){
        threads[i] = std::thread(std::move(tasks[i]), task_id_to_depot[i]);
    }

    vector<vector<Route>> new_routes_parallel(vehicle_groups.size());
    for (int i = 0; i < tasks.size(); i++){
        threads[i].join();
        new_routes_parallel[i] = futures[i].get();
    }

    // Regroup all the routes in a single vector
    vector<Route> new_routes;
    for (auto routes : new_routes_parallel){
        new_routes.insert(new_routes.end(), routes.begin(), routes.end());
    }

    return new_routes;
}
