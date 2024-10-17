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
#include "data_analysis/plot.h"

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
        {"ng", NG_STANDARD},
        {"dssr", DSSR_STANDARD},
        {"use_visited", true},
        {"pathwyse_time_limit", 0.0},
        {"switch_to_cyclic_price", true},
        {"delta ", 50},
        {"solution_pool_size", 10},
        {"alpha", 0.5},
        {"use_stabilisation", false},
        {"pricing_function", PRICING_PATHWYSE_BASIC},
        {"pricing_verbose", false}
    });

    std::map<int, string> NG_DSSR_MODES = {
        {NG_OFF, "off"},
        {NG_RESTRICTED, "restricted"},
        {NG_STANDARD, "standard"}
    };

    for (const auto& name : SMALL_INSTANCES){
        // Parse the instance from a JSON file
        auto start = chrono::steady_clock::now();
        cout << "-----------------------------------" << endl;
        string filename = "../data/" + name + ".json";
        cout << "Parsing the instance " << name << " from " << filename << endl;
        Instance instance = parse_file(filename, name, SMALL_SIZE, false);

        preprocess_interventions(instance);

        // Initialize the figure
        using namespace matplot;
        figure(true);
        // Set the height and width of the figure
        gcf()->width(2000);
        gcf()->height(800);
        auto ax1 = subplot(1, 2, 0);
        auto ax2 = subplot(1, 2, 1);
        title(ax1, "Objective value over time for small instances");
        xlabel(ax1, "Time (s)");
        ylabel(ax1, "Objective value (log scale)");
        title(ax2, "Objective value over time for small instances - first 15 values excluded");
        xlabel(ax2, "Time (s)");
        ylabel(ax2, "Objective value");
        sgtitle("Objective value over time - " + name);
        gcf()->title_font_size_multiplier(1.5);
        // Set the axis size to the maximum time
        xlim(ax1, {0, TIME_LIMIT * 1.2});
        xlim(ax2, {0, TIME_LIMIT * 1.2});
        hold(ax1, on);
        hold(ax2, on);

        for (const auto& use_max_formulation : {false, true}) {
            for (const auto& dssr_mode : {DSSR_RESTRICTED, DSSR_STANDARD}) {
                for (const auto& ng_mode : {NG_OFF, NG_RESTRICTED, NG_STANDARD}) {
                    vector<Route> routes;
                    cout << "-----------------------------------" << endl;
                    cout << "Current parameters : ";
                    cout << " Maximisation formulation : " << use_max_formulation;
                    cout << " - DSSR mode : " << NG_DSSR_MODES[dssr_mode];
                    cout << " - NG mode : " << NG_DSSR_MODES[ng_mode] << endl;
                    routes = vector<Route>();
                    routes.push_back(EmptyRoute(instance.nodes.size()));
                    cout << "Starting the column generation algorithm" << endl;
                    parameters.max_resources_dominance = instance.capacities_labels.size() + 1;
                    parameters.use_maximisation_formulation = use_max_formulation;
                    parameters.ng = ng_mode;
                    parameters.dssr = dssr_mode;

                    CGResult result = full_cg_procedure(instance, routes, parameters);

                    // Plot the objective value over time
                    string plot_name = use_max_formulation ? "Max" : "Min";
                    plot_name += " - DSSR=" + NG_DSSR_MODES[dssr_mode];
                    plot_name += " - NG=" + NG_DSSR_MODES[ng_mode];
                    // Add the final objective value to the plot name -only first 8 digits
                    if (use_max_formulation){
                        plot_name += " - Obj= " + to_string(convert_min_max_objective(result.objective_values, instance).back()).substr(0, 8);
                    } else {
                        plot_name += " - Obj= " + to_string(result.objective_values.back()).substr(0, 8);
                    }
                    std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
                    if (use_max_formulation){
                        plot_objective_values(ax1, result.objective_time_points, convert_min_max_objective(result.objective_values, instance), plot_name, true, 0);
                        plot_objective_values(ax2, result.objective_time_points, convert_min_max_objective(result.objective_values, instance), plot_name, false, 15);
                    } else {
                        plot_objective_values(ax1, result.objective_time_points, result.objective_values, plot_name, true, 0);
                        plot_objective_values(ax2, result.objective_time_points, result.objective_values, plot_name, false, 15);
                    }

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
                }
            }
        }

        // Set the legend
        legend(ax1);
        legend(ax2);

        // Save the plot
        string plot_filename = "../results/plots/" + name + "_pathwyse_basic_small.png";
        save(plot_filename);
    }


    return 0;
}
