#include "src/compact_solver.h"
#include "src/instance.h"
#include "src/parser.h"
#include "src/preprocessing.h"

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <memory>

int main(int argc, char** argv) {
    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    string default_filename = "../data/instance_1_all_feasible.json";
    Instance instance = parse_file(default_filename);

    preprocess_interventions(instance);
    auto end_parse = chrono::steady_clock::now();
    int diff_parse = chrono::duration_cast<chrono::milliseconds>(end_parse - start_parse).count();

    // Solve the problem using the compact formulation
    auto start_solve = chrono::steady_clock::now();
    compact_solver(instance);
    auto end_solve = chrono::steady_clock::now();


    return 0;
}