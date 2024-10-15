#include <matplot/matplot.h>

#include "../nlohmann/json.hpp"

#include <vector>


int main(int argc, char *argv[]){
    using namespace matplot;
    using json = nlohmann::json;
    using std::vector, std::string, std::pair;

    // List of files to load the data from
    vector<string> files = {
        "instance_1_2024-10-15-09-12-45",
        "instance_1_2024-10-15-09-18-09",
        "instance_1_2024-10-15-09-24-12",
        "instance_1_2024-10-15-09-30-48"
    };   

    hold(on);

    for (const string& file : files){
        // Load the data from the file
        string filename = "../results/" + file + ".json";
        std::ifstream i(filename);
        json data = json::parse(i);
        // X axis is the time
        vector<int> timepoints = data["evolution"]["time_points"];
        // Divide by 1000 to get seconds
        vector<double> timepoints_seconds;
        for (int t : timepoints){
            timepoints_seconds.push_back(t / 1000.);
        }
        vector<double> values = data["evolution"]["objective_values"];
        // Get the name from the rest of the data
        string name = data["instance"]["name"];
        name += " - " + data["parameters"]["pricing_function"].get<string>();
        if (data["parameters"]["stabilisation"]["use_stabilisation"]){
            double alpha = data["parameters"]["stabilisation"]["alpha"];
            name += " - ɑ=" + std::to_string(alpha);
        }
        // Replace "_" by " "
        std::replace(name.begin(), name.end(), '_', ' ');
        // Plot the data
        semilogy(timepoints_seconds, values)
            ->display_name(name);
    }

    xlabel("Time (s)");
    ylabel("Objective value (log scale)");
    title("Solution quality over time (minimization)");
    // Show the title
    legend();
    show();



    return 0;
}