#include "full_pricing.h"

#include "pricing_problem/subproblem.h"
#include "routes/route.h"
#include "routes/route_optimizer.h"
#include "clustering/clustering.h"
#include "tabu.h"

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
    bool using_cyclic_pricing,
    int n_ressources_dominance
    ){
    using std::vector;
    using std::packaged_task;
    using std::future;
    
    auto single_pricer = [&](int v){
            return solve_pricing_problem(instance, instance.vehicles.at(v), solution, using_cyclic_pricing, n_ressources_dominance);
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
            Route new_route = solve_pricing_problem(instance, vehicle, solution, using_cyclic_pricing, n_ressources_dominance);
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


std::vector<Route> full_pricing_problems_clustering(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> &vehicle_order,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    int seed
    ){
    using std::vector;
    using std::unique_ptr;

    // Randomize the seed
    if (seed == RANDOM_SEED){
        seed = std::chrono::system_clock::now().time_since_epoch().count();
    }


    // Initialize the new routes
    vector<Route> new_routes = {};

    // Generate clusters
    auto clusters = optimal_2_clustering(instance.similarity_matrix);
    // Perform 5 swaps
    for (int i = 0; i < 1; i++){
        clusters = greedy_neighbor(instance.similarity_matrix, clusters, i + seed);
    }

    // Use the previous function to solve the pricing problems
    for (auto cluster : clusters){
        vector<int> vehicle_order = cluster;
        vector<Route> new_routes_cluster = full_pricing_problems_diversification(solution, instance, vehicle_order, using_cyclic_pricing, n_ressources_dominance);
        new_routes.insert(new_routes.end(), new_routes_cluster.begin(), new_routes_cluster.end());
    }

    return new_routes;
}


std::vector<Route> full_pricing_problems_tabu_search(
    const DualSolution & solution,
    const Instance & instance,
    bool using_cyclic_pricing,
    int n_ressources_dominance,
    const std::vector<int> &vehicle_order,
    int max_iterations,
    int max_modifications,
    int seed
    ){
    using std::vector, std::unique_ptr;

    vector<Route> new_routes = {};

    for (int v : vehicle_order){
        // Compute a first route using the pricing problem
        const Vehicle& vehicle = instance.vehicles.at(v);
        Route new_route = solve_pricing_problem(instance, vehicle, solution, using_cyclic_pricing, n_ressources_dominance);
        // If this route is empty, we skip it
        if (new_route.id_sequence.size() <= 2){
            continue;
        }        
        // Then, we perform the tabu search
        vector<Route> tabu_list = tabu_search(new_route, max_iterations, max_modifications, solution, vehicle, instance);

        // We add the new routes to the list of new routes
        new_routes.insert(new_routes.end(), tabu_list.begin(), tabu_list.end());
    }

    return new_routes;
}


std::vector<Route> full_pricing_problems_basic_pulse(
    const DualSolution & solution,
    const Instance & instance,
    const std::vector<int> & vehicle_order,
    int delta,
    int pool_size
){
    using std::vector;
    using std::packaged_task;
    using std::future;
    
    auto single_pricer = [&](int v){
            return solve_pricing_problem_pulse(instance, instance.vehicles.at(v), solution, delta, pool_size);
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
        threads[i].join(); // Non-parallel execution
    }

    for (int i = 0; i < tasks.size(); i++){
        //threads[i].join();
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
    int delta,
    int pool_size
) {
    using std::vector;
    using std::packaged_task;
    using std::future;

    auto single_pricer = [&](int id){
            return solve_pricing_problem_pulse_grouped(instance, vehicle_groups.at(id), solution, delta, pool_size);
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
        threads[i].join(); // Non-parallel execution
    }

    vector<vector<Route>> new_routes_parallel(vehicle_groups.size());
    for (int i = 0; i < tasks.size(); i++){
        //threads[i].join();
        new_routes_parallel[i] = futures[i].get();
    }

    // Regroup all the routes in a single vector
    vector<Route> new_routes;
    for (auto routes : new_routes_parallel){
        new_routes.insert(new_routes.end(), routes.begin(), routes.end());
    }

    return new_routes;
}
