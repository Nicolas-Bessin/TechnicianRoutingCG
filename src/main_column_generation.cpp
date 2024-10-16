#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"
#include "master_problem/master_solver.h"
#include "master_problem/node.h"

#include "pricing_problem/subproblem.h"

#include "routes/route_optimizer.h"

#include "repair/repair.h"

#include "algorithms/column_generation.h"
#include "algorithms/full_procedure.h"

#include "data_analysis/analysis.h"
#include "data_analysis/export.h"

#include <matplot/matplot.h>

#include <memory>   
#include <iostream>
#include <iomanip>
#include <chrono>
#include <format>

inline constexpr int TIME_LIMIT = 60;
inline constexpr bool VERBOSE = true;
inline constexpr bool EXPORT_SOLUTION = true;

int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::tuple, std::set;
    using std::unique_ptr;
    namespace chrono = std::chrono;    

    int MAX_RESOURCES_DOMINANCE = -1;
    // Create the parameters for the column generation algorithm
    ColumnGenerationParameters parameters = ColumnGenerationParameters({
        {"time_limit", TIME_LIMIT},
        {"reduced_cost_threshold", 1e-6},
        {"verbose", false},
        {"max_iterations", 1000},
        {"max_consecutive_non_improvement", 5},
        {"compute_integer_solution", true},
        {"use_maximisation_formulation", false},
        {"max_resources_dominance", MAX_RESOURCES_DOMINANCE},
        {"switch_to_cyclic_price", true},
        {"delta ", 50},
        {"solution_pool_size", 10},
        {"alpha", 0.5},
        {"use_stabilisation", false},
        {"pricing_function", PRICING_PATHWYSE_BASIC},
        {"pricing_verbose", false}
    });


    vector<string> PRICING_FUNCTIONS = {
        PRICING_PATHWYSE_BASIC
    };

    for (const auto& name : SMALL_INSTANCES){
        // Parse the instance from a JSON file
        auto start = chrono::steady_clock::now();
        cout << "-----------------------------------" << endl;
        string filename = "../data/" + name + ".json";
        Instance instance = parse_file(filename, name, SMALL_SIZE, VERBOSE);

        preprocess_interventions(instance);

        // Initialize the figure
        using namespace matplot;
        figure(true);
        // Set the height and width of the figure
        gcf()->width(1200);
        gcf()->height(800);
        title("Objective value over time for small instances");
        xlabel("Time (s)");
        ylabel("Objective value");
        // Set the axis size to the maximum time
        xlim({0, TIME_LIMIT * 1.2});
        hold(on);

        for (const auto& algo_name : PRICING_FUNCTIONS) {
            // --------- MIN FORMULATION ---------
            vector<Route> routes;
            cout << "Initializing the routes with an empty route" << endl;
            routes = vector<Route>();
            routes.push_back(EmptyRoute(instance.nodes.size()));
            cout << "Starting the column generation algorithm" << endl;
            parameters.max_resources_dominance = instance.capacities_labels.size() + 1;
            parameters.pricing_function = algo_name;
            parameters.use_maximisation_formulation = false;

            CGResult result = full_cg_procedure(instance, routes, parameters);

            // Plot the objective value over time
            string plot_name = instance.name + " - " + parameters.pricing_function;
            if (parameters.use_stabilisation){
                plot_name += " - α=" + std::to_string(parameters.alpha);
            }
            std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
            vector<double> time_points_sec(result.objective_time_points.size());
            for (int i = 0; i < result.objective_time_points.size(); i++){
                time_points_sec[i] = result.objective_time_points[i] / 1000.0;
            }
            semilogy(time_points_sec, result.objective_values)
                ->display_name(plot_name);


            if (EXPORT_SOLUTION){
                // Dump the results to a file
                // Append the time to the filename
                const auto now = chrono::zoned_time(std::chrono::current_zone(), chrono::system_clock::now());
                string date = std::format("{:%Y-%m-%d-%H-%M-%OS}", now);
                string output_filename = "../results/" + name + "_" + date + ".json";
                export_solution(
                    output_filename, 
                    instance, 
                    result,
                    routes, 
                    parameters
                );
            }

            // --------- MAX FORMULATION ---------
            cout << "Initializing the routes with an empty route" << endl;
            routes = vector<Route>();
            routes.push_back(EmptyRoute(instance.nodes.size()));
            cout << "Starting the column generation algorithm" << endl;
            parameters.max_resources_dominance = instance.capacities_labels.size() + 1;
            parameters.pricing_function = algo_name;
            parameters.use_maximisation_formulation = true;

            result = full_cg_procedure(instance, routes, parameters);

            // Plot the objective value over time
            plot_name = instance.name + " - " + parameters.pricing_function + " - Maximisation";
            if (parameters.use_stabilisation){
                plot_name += " - α=" + std::to_string(parameters.alpha);
            }
            std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
            time_points_sec = vector<double>(result.objective_time_points.size());
            for (int i = 0; i < result.objective_time_points.size(); i++){
                time_points_sec[i] = result.objective_time_points[i] / 1000.0;
            }
            semilogy(time_points_sec, convert_min_max_objective(result.objective_values, instance))
                ->display_name(plot_name);
        }

        // Set the legend
        legend();

        // Save the plot
        string plot_filename = "../results/plots/" + name + "_objective_value.png";
        save(plot_filename);
    }


    return 0;
}
