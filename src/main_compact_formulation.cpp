#include "instance/instance.h"
#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "compact_formulation/compact_solver.h"

#include "compact_formulation/solution_converter.h"

#include "data_analysis/analysis.h"

#include "../nlohmann/json.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <memory>

inline constexpr std::string INSTANCE_FILE = "instance_1";
inline constexpr int N_INTERVENTIONS = 25;

inline constexpr int TIME_LIMIT = 1200;
inline constexpr bool VERBOSE = true;


int main(int argc, char** argv) {
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::map;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    const string size = "small";

    int intervention_size = -1;
    int vehicle_size = -1;
    std::array<string, 5> instances;

    map<string, map<string, double>> relaxed_values;

    for (const auto& size : {"small", "medium", "large"}){
        if (size == "small"){
            intervention_size = SMALL_INTERVENTIONS;
            vehicle_size = SMALL_VEHICLES;
            instances = SMALL_INSTANCES;
        } else if (size == "medium"){
            intervention_size = MEDIUM_INTERVENTIONS;
            vehicle_size = MEDIUM_VEHICLES;
            instances = MEDIUM_INSTANCES;
        } else if (size == "large"){
            intervention_size = LARGE_INTERVENTIONS;
            vehicle_size = LARGE_VEHICLES;
            instances = LARGE_INSTANCES;
        } else {
            cout << "Invalid size" << endl;
            return 1;
        }

        for (const auto& name : instances){
            // Parse the instance from a JSON file
            auto start = chrono::steady_clock::now();
            cout << "-----------------------------------" << endl;
            string filename = "../data/" + name + ".json";
            cout << "Parsing the instance " << name << " from " << filename << endl;
            Instance instance = parse_file(filename, name, intervention_size, vehicle_size, false);

            preprocess_interventions(instance);

            // Get the relaxed compact solution
            CompactSolution<double> relaxed_solution = relaxed_compact_solver(instance, TIME_LIMIT, VERBOSE);

            relaxed_values[size][name] = relaxed_solution.objective_value;
        }
    }

    // Export the relaxed values as a JSON file
    nlohmann::json relaxed_json = relaxed_values;
    std::ofstream relaxed_file("../results/relaxed_values_compact.json");
    relaxed_file << relaxed_json.dump(4);
    relaxed_file.close();



    return 0;
}