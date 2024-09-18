#include "genetic_solver.h"

#include <vector>
#include <array>
#include <random>
#include <bits/stdc++.h>

#include "algorithms/heuristics.h"
#include "repair/repair.h"

double evaluate(const GeneticSolution& solution, const Instance& instance) {
    double objective = 0;
    for (const auto& [v, route] : solution.routes){
        objective += instance.M * route.total_duration - route.total_cost;
    }
    return objective;
}

void repair(GeneticSolution& solution, const Instance& instance) {
    // Each intervention can be covered at most twice
    // This is because each parent represents a feasible solution - so each intervention is covered at most once per parent
    // Thus among the two parents, there are at most two routes that cover the same intervention
    
    // First step is seeing which interventions are covered twice, and which routes they are covered by
    std::map<int, std::array<int, 2>> intervention_to_routes;
    for (int i = 0; i < instance.number_interventions; i++){
        intervention_to_routes[i] = {-1, -1};
        for (auto [v, route] : solution.routes){
            if (route.is_in_route[i]){
                if (intervention_to_routes[i][0] == -1){
                    intervention_to_routes[i][0] = v;
                } else {
                    intervention_to_routes[i][1] = v;
                }
            }
        }
    }

    // Now we go through the interventions covered twice and see which one is the most advantageous to remove
    // For now, we do our removal by removing where the relative increase in route objective coefficient is the biggest
    for (const auto& [intervention, indexes] : intervention_to_routes){
        if (indexes[0] == -1 || indexes[1] == -1) continue;
        // Get the delta in route cost if we remove the intervention from each route
        double delta1 = compute_delta(solution.routes[indexes[0]], intervention, instance);
        double delta2 = compute_delta(solution.routes[indexes[1]], intervention, instance);
        // Get the biggest relative increase in route objective coefficient 
        // Equivalent to the biggest delta in route cost
        double relative1 = delta1 / solution.routes[indexes[0]].total_cost;
        double relative2 = delta2 / solution.routes[indexes[1]].total_cost;
        if (relative1 > relative2){
            delete_intervention(solution.routes[indexes[0]], intervention, instance);
        } else {
            delete_intervention(solution.routes[indexes[1]], intervention, instance);
        }
    }

    // If after the deletion, we get an empty route, we remove it
    for (auto it = solution.routes.begin(); it != solution.routes.end();){
        if (it->second.id_sequence.size() == 2){
            it = solution.routes.erase(it);
            //std::cout << "Empty route removed" << std::endl;
        } else {
            it++;
        }
    }
}



GeneticSolution crossover(const GeneticSolution& parent1, const GeneticSolution& parent2, const Instance& instance) {
    using std::vector;
    // Enumerate through all the vehicles in the instance, and if two are available, pick a random parent to get the route from
    GeneticSolution child;
    for (int v = 0; v < instance.number_vehicles; v++){
        // If both parents have the vehicle, pick one at random
        if (parent1.routes.count(v) && parent2.routes.count(v)){
            // Generate a random number between 0 and 1
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0, 1);
            double random = dis(gen);
            if (random < 0.5){
                child.routes[v] = parent1.routes.at(v);
            } else {
                child.routes[v] = parent2.routes.at(v);
            }
        } else if (parent1.routes.count(v)){  
            child.routes[v] = parent1.routes.at(v);
        } else if (parent2.routes.count(v)){
            child.routes[v] = parent2.routes.at(v);
        }
    }
    // First thing : we need to repair the solution
    repair(child, instance);
    // Evaluate the objective value of the child
    child.objective = evaluate(child, instance);

    return child;
}


GeneticSolution greedy_initializer(const Instance& instance) {
    using std::vector;
    // First step : generate a random permutation of [|1, n_v|]
    vector<int> order(instance.number_vehicles);
    std::iota(order.begin(), order.end(), 0);
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(order.begin(), order.end(), g);

    // Second step : generate the routes
    vector<Route> routes = greedy_heuristic(instance, order);

    // Third step : build the GeneticSolution
    GeneticSolution solution;
    for (const Route& route : routes){
        solution.routes[route.vehicle_id] = route;
    }
    solution.objective = evaluate(solution, instance);

    return solution;
}



void genetic_algorithm(const Instance& instance) {
    using std::vector;
    using std::cout, std::endl;

    // Define the population size
    int population_size = 50;
    int increase_population = 200;
    int reduce_by = 175;
    // Define the number of generations
    int number_generations = 100;

    double best_objective = -1e9;
    double current_gen_best = -1e9;

    // Initialize the population
    vector<GeneticSolution> population;
    for (int i = 0; i < population_size; i++){
        population.push_back(greedy_initializer(instance));
    }

    // Update the initial best objective
    for (const GeneticSolution& solution : population){
        if (solution.objective > best_objective){
            best_objective = solution.objective;
        }
    }
    cout << "Initial best objective : " << best_objective << endl;

    // Perform the genetic algorithm
    for (int generation = 0; generation < number_generations; generation++){
        // Perform the crossover
        int old_population_size = population.size();
        cout << "Generation " << generation << " - Population size : " << old_population_size << endl;
        // Pick two parents at random increase_population times
        for (int i = 0; i < increase_population; i++){
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, old_population_size - 1);
            GeneticSolution parent1 = population[dis(gen)];
            GeneticSolution parent2 = population[dis(gen)];
            // Perform the crossover
            GeneticSolution child = crossover(parent1, parent2, instance);
            // Add the child to the population
            population.push_back(child);
        }

        // Sort the population by objective value
        std::sort(population.begin(), population.end(), [&instance](const GeneticSolution& a, const GeneticSolution& b){
            return a.objective > b.objective;
        });
        // Update the generation's best objective
        current_gen_best = population[0].objective;

        // Keep the best solution
        if (population[0].objective > best_objective){
            best_objective = population[0].objective;
        }
    

        // Keep only the best population_size solutions
        population.resize(population.size() - reduce_by);

        cout << "End of generation " << generation << " - Best objective : " << best_objective;
        cout << " - Current generation best : " << current_gen_best << endl;
    }
}