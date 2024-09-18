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
#define VERBOSE true
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

    vector<Route> routes = vector<Route>();
    routes.push_back(EmptyRoute(instance.nodes.size()));
    auto root_node = RootNode(routes);

    auto interger_solution = relaxed_RMP(instance, routes, root_node);


    // Manually compute the expected objective value
    double expected_objective = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        expected_objective += instance.nodes[i].duration * instance.M;
    }

    cout << "Expected objective value : " << expected_objective << endl;
    cout << "Objective value : " << interger_solution.objective_value << endl;
}
