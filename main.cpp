#include "src/parser.h"
#include "src/preprocessing.h"
#include "src/column_generation.h"
#include "src/analysis.h"
#include "pathwyse/core/solver.h"

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>

#include <algorithm>
#include <random>

#define SCALE_FACTOR 1
#define TIME_LIMIT 30
#define THRESHOLD 1e-6
#define VERBOSE true

void solution_analysis(const Instance & instance, const CGResult & result ) {
    using std::cout, std::endl;

    cout << "-----------------------------------" << endl;
    int iteration = result.number_of_iterations;
    int master_time = result.master_time;
    int pricing_time = result.pricing_time;
    int integer_time = result.integer_time;
    cout << "Total time spent solving the master problem : " << master_time << " ms" << endl;
    cout << "Total time spent solving the pricing sub problems : " << pricing_time << " ms - average time : " << pricing_time / iteration << " ms" << endl;
    cout << "Total time spent solving the integer problem : " << integer_time << " ms" << endl;

    cout << "Total running time : " << master_time + pricing_time + integer_time << " ms" << endl;

    cout << "-----------------------------------" << endl;

    // Solution analysis
    cout << "Number of covered interventions : " << count_covered_interventions(result.integer_solution, result.routes, instance);
    cout << " / " << instance.number_interventions << endl;

    cout << "Number of used vehicles : " << count_used_vehicles(result.integer_solution, result.routes, instance);
    cout << " / " << instance.vehicles.size() << endl;

    cout << "Number of interventions that could be covered : " << count_coverable_interventions(result.integer_solution, result.routes, instance) << endl;

    // Check that all used result.routes are feasible
    bool all_feasible = true;
    for (int i = 0; i < result.routes.size(); i++){
        const Route& route = result.routes.at(i);
        if (result.integer_solution.coefficients[i] > 0 && !is_route_feasible(route, instance)){
            cout << "Route " << i << " is not feasible" << endl;
            all_feasible = false;
        }
    }
    if (all_feasible){
        cout << "All routes are feasible" << endl;
    }

    cout << "Number of routes with duplicates : " << count_routes_with_duplicates(result.routes) << " / " << result.routes.size() << endl;
    cout << "Number of used routes with duplicates : " << count_used_routes_with_duplicates(result.integer_solution, result.routes) << endl;

    cout << "Number of route kilometres : " << count_kilometres_travelled(result.integer_solution, result.routes, instance) << " km" << endl;

    cout << "Time spent travelling : " << time_spent_travelling(result.integer_solution, result.routes, instance) << " minutes" << endl;
    cout << "Time spent working : " << time_spent_working(result.integer_solution, result.routes, instance) << " minutes" << endl;
    cout << "Time spent waiting : " << time_spent_waiting(result.integer_solution, result.routes, instance) << " minutes" << endl;

    cout << "-----------------------------------" << endl;
    //print_used_routes(integer_solution, result.routes, instance); 
    print_non_covered_interventions(result.integer_solution, result.routes, instance, false);
    cout << "-----------------------------------" << endl;
    print_used_vehicles(result.integer_solution, result.routes, instance);
    print_vehicles_non_covered(result.integer_solution, result.routes, instance);
    cout << "-----------------------------------" << endl;

}

int main(int argc, char *argv[]){

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

    
    cout << "Total time spent parsing the instance : " << diff_parse << " ms" << endl;

    // Create a dummy route for the first vehicle
    vector<Route> routes;
    routes.push_back(Route(0, instance.number_interventions));

    // Do a first round of column generation
    CGResult initial_cg = column_generation(instance, routes, THRESHOLD, TIME_LIMIT, true);
    
    // Analyze the solution
    solution_analysis(instance, initial_cg);
    

    return 0;
}



