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
        std::string ax1_title = "Solution cost over time";
        title(ax1, ax1_title);
        xlabel(ax1, "Time (s)");
        ylabel(ax1, "Solution cost ");
        string ax2_title = "Number of covered interventions over time";
        title(ax2, ax2_title);
        xlabel(ax2, "Time (s)");
        ylabel(ax2, "# Covered interventions");

        // Set the axis size to the maximum time
        xlim(ax1, {0, TIME_LIMIT * 1.2});
        xlim(ax2, {0, TIME_LIMIT * 1.2});
        hold(ax1, on);
        hold(ax2, on);

        for (const auto& use_naive_M : {false, true}) {
                vector<Route> routes;
                cout << "-----------------------------------" << endl;
                cout << "Current parameters : ";
                cout << " Using naive M : " << use_naive_M << endl;
                routes = vector<Route>();
                routes.push_back(EmptyRoute(instance.nodes.size()));
                cout << "Starting the column generation algorithm" << endl;
                parameters.max_resources_dominance = instance.capacities_labels.size() + 1;

                if (use_naive_M){
                    instance.M = compute_M_naive(instance);
                } else {
                    instance.M = compute_M_perV(instance);
                }

                CGResult result = full_cg_procedure(instance, routes, parameters);

                // Plot the objective value over time
                string plot_name;
                if (use_naive_M){
                    plot_name = "Naive M = " + to_string(instance.M);
                } else {
                    plot_name = "Standard M = " + to_string(instance.M);
                }
                // Add the final objective value to the plot name - only first 8 digits      
                plot_name += " - Cost = " + to_string(result.objective_values.back()).substr(0, 8);
                std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
                plot_objective_values(ax1, result.time_points, result.solution_costs, plot_name, false, 0);
                plot_objective_values(ax2, result.time_points, result.covered_interventions, plot_name, false, 0);
                

                if (EXPORT_SOLUTION){
                    // Dump the results to a file & append the time to the filename
                    const auto now = chrono::zoned_time(std::chrono::current_zone(), chrono::system_clock::now());
                    string date = std::format("{:%Y-%m-%d-%H-%M-%OS}", now);
                    string output_filename = "../results/solution_cost/" + size + "_" + name + "_" + date + ".json";
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
        legend(ax1);
        legend(ax2);

        // Save the plot
        string plot_filename = "../results/plots/solution_cost_" + size + "_" + name + ".png";
        save(plot_filename);
    }


    return 0;
}
