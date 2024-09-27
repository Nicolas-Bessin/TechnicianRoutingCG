#include "export.h"

#include "analysis.h"

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

void export_solution(const std::string& filename, const Instance& instance, const IntegerSolution& solution, const std::vector<Route>& routes, int elapsed_time){

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

    j["solution"] = {
        {"objective_value", compute_integer_objective(solution, routes, instance)},
        {"number_covered_interventions", count_covered_interventions(solution, routes, instance)},
        {"number_used_vehicles", count_used_vehicles(solution, routes, instance)},
        {"total_fixed_cost", fixed_cost(solution, routes, instance)},
        {"total_working_time", time_spent_working(solution, routes, instance)},
        {"total_travel_time", time_spent_travelling(solution, routes, instance)},
        {"total_waiting_time", time_spent_waiting(solution, routes, instance)},
        {"total_kilometres_travelled", count_kilometres_travelled(solution, routes, instance)},
        {"time_to_compute", elapsed_time}
    };

    // Add the routes
    j["routes"] = json::array();
    for (int i = 0; i < routes.size(); i++){
        if (solution.coefficients[i] > 0){
            j["routes"].push_back(export_route(routes[i], instance));
        }
    }

    // Write the json to a file
    std::string output = j.dump(4);
    std::ofstream out(filename);
    out << output;
    out.close();

}