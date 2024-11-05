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
#include <filesystem>

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
        {"compute_intermediate_integer_solutions", true},
        {"use_maximisation_formulation", false},
        {"max_resources_dominance", MAX_RESOURCES_DOMINANCE},
        {"ng", NG_STANDARD},
        {"dssr", DSSR_STANDARD},
        {"use_visited", true},
        {"bidirectional_DP", false},
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

    const string size = "small";

    int intervention_size = -1;
    int vehicle_size = -1;
    std::array<string, 5> instances;

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
    // Subfolder for the results
    string subfolder = "bigMcomparisons/";
    // If the subfolder does not exist, create it
    if (!std::filesystem::exists("../results/plots/" + subfolder)){
        std::filesystem::create_directory("../results/plots/" + subfolder);
    }
    if (!std::filesystem::exists("../results/" + subfolder)){
        std::filesystem::create_directory("../results/" + subfolder);
    }

    for (const auto& name : instances){
        // Parse the instance from a JSON file
        auto start = chrono::steady_clock::now();
        cout << "-----------------------------------" << endl;
        string filename = "../data/" + name + ".json";
        cout << "Parsing the instance " << name << " from " << filename << endl;
        Instance instance = parse_file(filename, name, intervention_size, vehicle_size, false);

        preprocess_interventions(instance);
        int exclude_from_linear_plot = 15;

        // Initialize the figure
        using namespace matplot;
        figure(true);
        string fig_title = "Current best solution cost over time - " + name + " - " + size;
        std::replace(fig_title.begin(), fig_title.end(), '_', ' ');
        sgtitle(fig_title);
        gcf()->title_font_size_multiplier(1.5);
        // Set the height and width of the figure
        gcf()->width(2000);
        gcf()->height(800);

        auto ax1 = subplot(1, 2, 0);
        auto ax2 = subplot(1, 2, 1);
        std::string ax1_title = "Number of covered interventions value and intermediary integer solutions every 5 iterations";
        title(ax1, ax1_title);
        xlabel(ax1, "Time (s)");
        ylabel(ax1, "Number of covered interventions ");
        string ax2_title = "Solution cost value and intermediary integer solutions every 5 iterations";
        title(ax2, ax2_title);
        xlabel(ax2, "Time (s)");
        ylabel(ax2, "Solution cost");

        // Set the axis size to the maximum time
        xlim(ax1, {0, TIME_LIMIT * 1.2});
        xlim(ax2, {0, TIME_LIMIT * 1.2});
        hold(ax1, on);
        hold(ax2, on);

        for (const auto& current_M : {5.0, 10.0, compute_M_naive(instance), compute_M_perV(instance)}){
                vector<Route> routes;
                cout << "-----------------------------------" << endl;
                cout << "Current parameters : ";
                cout << " Using big M of " << current_M << " - ";
                routes = vector<Route>();
                routes.push_back(EmptyRoute(instance.nodes.size()));
                cout << "Starting the column generation algorithm" << endl;
                parameters.max_resources_dominance = instance.capacities_labels.size() + 1;
                instance.M = current_M;

                CGResult result = full_cg_procedure(instance, routes, parameters);

                // Plot the objective value over time
                string plot_name = name + " - M = " + std::to_string(current_M);
                std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
                plot_objective_values(ax1, result.time_points, result.covered_interventions, plot_name);
                plot_objective_values(ax1, result.time_points, result.integer_covered_interventions, plot_name + " - Integer");
                plot_objective_values(ax2, result.time_points, result.solution_costs, plot_name, false, 10);
                plot_objective_values(ax2, result.time_points, result.integer_solution_costs, plot_name + " - Integer", false, 10);
                
                

                if (EXPORT_SOLUTION){
                    // Dump the results to a file & append the time to the filename
                    const auto now = chrono::zoned_time(std::chrono::current_zone(), chrono::system_clock::now());
                    string date = std::format("{:%Y-%m-%d-%H-%M-%OS}", now);
                    string output_filename = "../results/" + subfolder + size + "_" + name + "_" + date + ".json";
                    export_solution(
                        output_filename, 
                        instance, 
                        result,
                        routes, 
                        parameters
                    );
                }
            }

        // Set the legend
        //legend(ax1);
        legend(ax2);

        // Save the plot
        string plot_filename = "../results/plots/" + subfolder + size + "_" + name + ".png";
        save(plot_filename);
    }


    return 0;
}
