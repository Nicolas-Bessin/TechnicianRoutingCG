#include "export.h"

#include "analysis.h"
#include "master_problem/master_solver.h"

#include <fstream>
#include "../../nlohmann/json.hpp"

nlohmann::json export_route(const Route& route, const Instance& instance) {
    using std::vector, std::string;

    nlohmann::json j;

    j["vehicle_id"] = route.vehicle_id;
    j["sequence"] = route.id_sequence;
    // Build a sequence of node ids 
    vector<string> sequence_ids;
    for (int i : route.id_sequence){
        sequence_ids.push_back(instance.nodes[i].id);
    }
    j["sequence_ids"] = sequence_ids;
    // Add the start times
    vector<int> start_times = compute_start_times(route, instance);
    j["start_times"] = start_times;
    // Finally the list of technicians
    j["technicians"] = instance.vehicles[route.vehicle_id].technicians;

    return j;
}

nlohmann::json export_parameters(const ColumnGenerationParameters& parameters) {
    using std::string;
    nlohmann::json j;

    // General parameters
    j["general"] = {
        {"time_limit", parameters.time_limit},
        {"reduced_cost_threshold", parameters.reduced_cost_threshold},
        {"max_iterations", parameters.max_iterations},
        {"max_consecutive_non_improvement", parameters.max_consecutive_non_improvement},
        {"compute_integer_solution", parameters.compute_integer_solution},
        {"use_maximisation_formulation", parameters.use_maximisation_formulation}
    };

    // Pathwyse related parameters
    j["pathwyse"] = {
        {"max_resources_dominance", parameters.max_resources_dominance},
        {"switch_to_cyclic_pricing", parameters.switch_to_cyclic_pricing},
        // Also export the relevant pathwyse parameters
        {"ng", parameters.ng},
        {"dssr", parameters.dssr},
        {"use_visited", parameters.use_visited},
        {"pathwyse_time_limit", parameters.pathwyse_TL}
    };

    // Pulse related parameters
    j["pulse"] = {
        {"delta", parameters.delta},
        {"solution_pool_size", parameters.solution_pool_size}
    };

    // Stabilisation parameters
    j["stabilisation"] = {
        {"alpha", parameters.alpha},
        {"use_stabilisation", parameters.use_stabilisation}
    };

    // Pricing function
    j["pricing_function"] = parameters.pricing_function;
    return j;
}

void export_solution(
    const std::string& filename, 
    const Instance& instance, 
    const CGResult& result,
    const std::vector<Route>& routes, 
    const ColumnGenerationParameters& parameters
    ){

    using namespace nlohmann;

    json j;

    // Add the instance information
    j["instance"] = {
        {"name", instance.name},
        {"number_interventions", instance.number_interventions},
        {"number_vehicles", instance.number_vehicles},
        {"outsource_cost", instance.M},
        {"cost_per_km", instance.cost_per_km}
    };

    int elapsed_time = (result.master_time + result.pricing_time) / 1000;
    j["solution"] = {
        {"objective_value", compute_integer_objective(result.integer_solution, routes, instance)},
        {"number_covered_interventions", count_covered_interventions(result.integer_solution, routes, instance)},
        {"number_used_vehicles", count_used_vehicles(result.integer_solution, routes, instance)},
        {"total_fixed_cost", fixed_cost(result.integer_solution, routes, instance)},
        {"total_working_time", time_spent_working(result.integer_solution, routes, instance)},
        {"total_travel_time", time_spent_travelling(result.integer_solution, routes, instance)},
        {"total_waiting_time", time_spent_waiting(result.integer_solution, routes, instance)},
        {"total_kilometres_travelled", count_kilometres_travelled(result.integer_solution, routes, instance)},
        {"time_to_compute", elapsed_time}
    };

    // Add the routes
    j["routes"] = json::array();
    for (int i = 0; i < routes.size(); i++){
        if (result.integer_solution.coefficients[i] > 0){
            j["routes"].push_back(export_route(routes[i], instance));
        }
    }

    // Add the parameters
    j["parameters"] = export_parameters(parameters);


    // Add the evolution of the relaxed RMP objective
    j["evolution"] = {
        {"objective_values", result.objective_values},
        {"time_points", result.time_points},
        {"solution_costs", result.solution_costs},
        {"covered_interventions", result.covered_interventions},
        {"integer_objective_values", result.integer_objective_values},
        {"integer_covered_interventions", result.integer_covered_interventions}
    };

    // Write the json to a file
    std::string output = j.dump(4);
    std::ofstream out(filename);
    out << output;
    out.close();

}