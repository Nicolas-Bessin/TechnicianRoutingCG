#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"
#include "master_problem/master_solver.h"
#include "master_problem/node.h"

#include "pricing_problem/subproblem.h"

#include "routes/route_optimizer.h"

#include "repair/repair.h"

#include "algorithms/column_generation.h"
#include "algorithms/sequential_column_generation.h"

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

inline constexpr int TIME_LIMIT = 300;
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
        {"verbose", false},
        {"compute_integer_solution", true},
        {"max_resources_dominance", MAX_RESOURCES_DOMINANCE},
    });

    const string size = "large";

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
    string subfolder = "seqVSBigM/";
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

        auto ax1 = subplot(1, 1, 0);
        std::string ax1_title = "Solution cost over time";
        title(ax1, ax1_title);
        xlabel(ax1, "Time (s)");
        ylabel(ax1, "Solution cost");
        xlim(ax1, {0, TIME_LIMIT * 1.2});
        hold(ax1, on);

        // auto ax2 = subplot(1, 2, 1);
        // string ax2_title = "Solution cost value and intermediary integer solutions every 5 iterations";
        // title(ax2, ax2_title);
        // xlabel(ax2, "Time (s)");
        // ylabel(ax2, "Solution cost");
        // xlim(ax2, {0, TIME_LIMIT * 1.2});
        // hold(ax2, on);

        vector<Route> routes;
        cout << "-----------------------------------" << endl;
        cout << " Using big M formulation" << endl;
        routes = vector<Route>();
        routes.push_back(EmptyRoute(instance.nodes.size()));
        parameters.max_resources_dominance = instance.capacities_labels.size() + 1;

        CGResult result = column_generation(instance, routes, parameters);

        vector<Route> routes_seq;
        cout << "-----------------------------------" << endl;
        cout << "Using sequential column generation" << endl;
        routes_seq.push_back(EmptyRoute(instance.nodes.size()));

        SequentialCGResult full_result_seq = sequential_column_generation(instance, routes_seq, parameters);
        CGResult result_seq = full_result_seq.combined_result;

        // Plot the objective value over time
        string plot_name = "";
        std::replace(plot_name.begin(), plot_name.end(), '_', ' ');
        plot_objective_values(ax1, result.time_points, result.solution_costs, plot_name + " - Big M", false, 1);
        plot_objective_values(ax1, result_seq.time_points, result_seq.solution_costs, plot_name + " - Sequential", false, 1);
        // Plot a vertical line separating the two phases
        double min_line = *std::min_element(result_seq.solution_costs.begin(), result_seq.solution_costs.end());
        double max_line = *std::max_element(result_seq.solution_costs.begin(), result_seq.solution_costs.end());
        line(ax1, full_result_seq.phase_1_time, min_line, full_result_seq.phase_1_time, max_line)
            ->line_style(":")
            .color("red")
            .line_width(1.5);
        
        

        if (EXPORT_SOLUTION){
            // Dump the results to a file & append the time to the filename
            const auto now = chrono::zoned_time(std::chrono::current_zone(), chrono::system_clock::now());
            string date = std::format("{:%Y-%m-%d-%H-%M-%OS}", now);
            string output_filename_bigM = "../results/" + subfolder + size + "_" + name + "_bigM_" + date + ".json";
            export_solution(
                output_filename_bigM, 
                instance, 
                result,
                routes, 
                parameters
            );
            string output_filename_seq = "../results/" + subfolder + size + "_" + name + "_seq_" + date + ".json";
            export_solution(
                output_filename_seq, 
                instance, 
                result_seq,
                routes_seq, 
                parameters
            );

        }
        

        // Set the legend
        legend(ax1);

        // Save the plot
        string plot_filename = "../results/plots/" + subfolder + size + "_" + name + ".png";
        save(plot_filename);
    }


    return 0;
}
