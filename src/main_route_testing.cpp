#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "clustering/clustering.h"


#include "master_problem/master.h"
#include "master_problem/rmp_solver.h"

#include "pricing_problem/pricing.h"
#include "pricing_problem/subproblem.h"

#include "data_analysis/analysis.h"
#include "data_analysis/plot.h"

#include "algorithms/heuristics.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <random>

#define TIME_LIMIT 120
#define SOLVER_MODE IMPOSE_ROUTING
#define THRESHOLD 1e-6
#define VERBOSE false
#define GREEDY_INIT false
#define N_INTERVENTIONS 25

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::tuple, std::set;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename, true);

    auto begin = chrono::steady_clock::now();
    // Create the pricing problems for each vehicle
    vector<unique_ptr<Problem>> pricing_problems;
    for (const Vehicle &vehicle : instance.vehicles){
        auto pricing_problem = create_pricing_instance(instance, vehicle);
        pricing_problems.push_back(std::move(pricing_problem));
    }
    auto end = chrono::steady_clock::now();
    int diff = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
    cout << "Total time spent building the pricing problems : " << diff << " ms" << endl;


}
