#include "instance/parser.h"
#include "instance/preprocessing.h"

#include "master_problem/master.h"

#include "algorithms/genetic_solver.cpp"

#include "data_analysis/analysis.h"

#define TIME_LIMIT 60


int main(int argc, char *argv[]){

    using std::cout, std::endl;
    using std::setprecision, std::fixed;
    using std::vector, std::string, std::to_string, std::pair;
    using std::unique_ptr;
    namespace chrono = std::chrono;

    // Parse the instance from a JSON file
    auto start_parse = chrono::steady_clock::now();
    cout << "Technician Routing Problem using Column Generation" << endl;
    cout << "-----------------------------------" << endl;
    string default_filename = "../data/instance_1.json";
    Instance instance = parse_file(default_filename, false);

    // Only keep the first 20 nodes
    vector<int> kept_nodes = vector<int>(instance.number_interventions);
    for (int i = 0; i < 25; i++){
        kept_nodes[i] = 1;
    }
    instance = cut_instance(instance, kept_nodes);

    preprocess_interventions(instance);


    genetic_algorithm(instance);

}
