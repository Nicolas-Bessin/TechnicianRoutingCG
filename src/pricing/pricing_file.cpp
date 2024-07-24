#include "pricing_file.h"
#include "../constants.h"
#include "../../pathwyse/core/solver.h"

#include <fstream>


void write_pricing_instance(const std::string &filepath, const Instance &instance, const Vehicle &vehicle) {
    using std::string, std::to_string;
    using std::ofstream, std::endl;

    ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file " + filepath);
    }
    // Define the keyword - value separator
    string separator = " : ";
    // We first set the name of the problem
    file << "NAME" << separator << "v_" << vehicle.id << endl;
    file << "COMMENT" << separator << "Pricing problem for vehicle " << vehicle.id << endl;
    // Now we will determine the key values of the problem
    int n_interventions = vehicle.interventions.size();
    int origin = n_interventions;
    int destination = origin + 1;
    int n_nodes = n_interventions + 1 + int(origin != destination);

    // Number of ressources is number of capacities + time window (+ node limit if we consider it)
    int n_capacities = instance.capacities_labels.size();
    int n_resources = n_capacities + 1;
    file << "SIZE" << separator << n_nodes << endl;
    file << "DIRECTED" << separator << 1 << endl;
    file << "CYCLIC" << separator << 0 << endl;
    file << "ORIGIN" << separator << origin << endl;
    file << "DESTINATION" << separator << destination << endl;
    file << "RESOURCES" << separator << n_resources << endl;
    file << "RES_NAMES" << separator;
    for (int i = 0; i < instance.capacities_labels.size(); i++) {
        file << i << " ";
    }
    file << endl;

    // We then describe the ressource types : first the time window, then the capacities
    file << "RES_TYPE" << endl;
    file << 0 << " " << "TW" << endl;
    for (int i = 0; i < n_capacities; i++) {
        file << i + 1 << " " << "CAP" << endl;
    }
    // file << n_resources << " " << "NODELIM" << endl;
    file << "END" << endl;

    // Then, we add the global bound on the resources (if applicable)
    file << "RES_BOUND" << endl;
    for (int k = 0; k < n_capacities; k++) {
        file << k+1 << " " << 0 << " " << vehicle.capacities.at(instance.capacities_labels[k]) << endl;
    }
    // file << n_resources - 1 << " " << 0 << " " << n_nodes << endl;
    file << "END" << endl;
    // We now describe the bounds for the time window on each node
    file << "RES_NODE_BOUND" << endl;
    for (int i = 0; i < n_interventions; i++) {
        const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
        double start_window = intervention.start_window;
        double end_window_arr = intervention.end_window - intervention.duration;
        // Adjust the end window : the intervention must end and leave enough time to go back to the depot
        double time_to_depot = metric(intervention, instance.nodes.at(vehicle.depot), instance.time_matrix);
        double end_window = std::min(end_window_arr, END_DAY - time_to_depot);
        file << 0 << " " << i << " " << start_window << " " << end_window << endl;
    }
    // We add the time window for the depot
    file << 0 << " " << origin << " " << 0 << " " << END_DAY << endl;
    file << 0 << " " << destination << " " << 0 << " " << END_DAY << endl;
    file << "END" << endl;

    // EDGE_COST : for each edge (i,j), we will set its cost as the cost of going from i to j
    file << "EDGE_COST" << endl;
    for (int i = 0; i < n_interventions; i++) {
        const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
        for (int j = 0; j < n_interventions; j++) {
            const Node &intervention_next = instance.nodes.at(vehicle.interventions.at(j));
            double distance = metric(intervention, intervention_next, instance.distance_matrix);
            double cost = distance * instance.cost_per_km;
            file << i << " " << j << " " << cost << endl;
        }
        // We add the cost of going from the intervention to the depot and vice versa
        double distance_to_depot = metric(intervention, instance.nodes.at(vehicle.depot), instance.distance_matrix);
        double cost_to_depot = distance_to_depot * instance.cost_per_km;
        file << i << " " << destination << " " << cost_to_depot << endl;
        file << origin << " " << i << " " << cost_to_depot << endl;
    }
    file << "END" << endl;

    // EDG_CONSUMPTION : for each edge (i,j), we will set the time consumption of going from i to j for resource 0 (time window)
    file << "EDGE_CONSUMPTION" << endl;
    for (int i = 0; i < n_interventions; i++) {
        const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
        for (int j = 0; j < n_interventions; j++) {
            const Node &intervention_next = instance.nodes.at(vehicle.interventions.at(j));
            double time = metric(intervention, intervention_next, instance.time_matrix);
            file << 0 << " " << i << " " << j << " " << time << endl;
        }
        // We add the time of going from the intervention to the depot and vice versa
        double time_to_depot = metric(intervention, instance.nodes.at(vehicle.depot), instance.time_matrix);
        file << 0 << " " << i << " " << destination << " " << time_to_depot << endl;
        file << 0 << " " << origin << " " << i << " " << time_to_depot << endl;
    }
    file << "END" << endl;

    // NODE_COST : the cost of a node is -M * duration (no consideration of alpha for now)
    // file << "NODE_COST" << endl;
    // for (int i = 0; i < n_interventions; i++) {
    //     const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
    //     int cost = - instance.M * intervention.duration;
    //     file << i << " " << cost << endl;
    // }
    // // We add the cost of the depot
    // int cost_depot = vehicle.cost;
    // file << origin << " " << cost_depot << endl;
    // file << "END" << endl;

    // NODE_CONSUMPTIONS : the consumption of each ressource for each node
    file << "NODE_CONSUMPTION" << endl;
    // Begin with the time window
    for (int i = 0; i < n_interventions; i++) {
        const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
        double duration = intervention.duration;
        file << 0 << " " << i << " " << duration << endl;
    }
    // Then the capacities
    for (int k = 0; k < n_capacities; k++) {
        for (int i = 0; i < n_interventions; i++) {
            const Node &intervention = instance.nodes.at(vehicle.interventions.at(i));
            file << k + 1 << " " << i << " " << intervention.quantities.at(instance.capacities_labels[k]) << endl;
        }
    }
    // No need to add consumptions for the node limit
    file << "END" << endl;




    file.close();
}
  


std::vector<Route> solve_pricing_problem_file(const std::string &filepath,
    const std::vector<double> &alphas, double beta, 
    const Instance &instance, const Vehicle &vehicle) {

    using std::vector, std::list;
    using std::cout, std::endl;
    // Create a solver object
    Solver solver = Solver("../pathwyse.set");

    // Read the problem from the file
    solver.readProblem(filepath);

    // Update the problem with the dual values
    Problem* problem = solver.getProblem();
    int n_nodes = problem->getNumNodes();
    int n_interventions = n_nodes - 2;
    int origin = problem->getOrigin();
    int destination = problem->getDestination();

    DefaultCost* objective = (DefaultCost*) problem->getObj();
    // Update the node costs
    for (int i = 0; i < n_interventions; i++) {
        int intervention_index = vehicle.interventions.at(i);
        double cost = alphas[intervention_index] - instance.M * instance.nodes.at(intervention_index).duration;
        objective->setNodeCost(i, cost);
    }

    // Update the constant term as a cost for the depot
    double cost_depot = vehicle.cost + beta;
    objective->setNodeCost(origin, cost_depot);

    // Solve the problem
    solver.setupAlgorithms();
    solver.solve();

    // If the problem is indeterminate (time limit reached, we return an empty vector)
    if (solver.getProblem()->getStatus() == PROBLEM_INDETERMINATE) {
        cout << "Time limit reached for vehicle " << vehicle.id << endl;
        return vector<Route>();
    }
    //solver->printBestSolution();
    // Get the solution we found
    vector<Path> solutions = solver.getBestSolutions(5);
    //cout << "Solver found " << solver->getNumberOfSolutions() << " solutions" << endl;
    // Convert each Path object to a Route object
    vector<Route> routes;
    for (auto& path : solutions) {
        // Get the sequence as a vector of integers
        double reduced_cost = - path.getObjective(); // (We got the max reduced cost by minimizing the opposite of the reduced cost)
        list<int> tour_list = path.getTour();
        vector<int> tour(tour_list.begin(), tour_list.end());
        // Check that we found an elementary path
        if (!path.isElementary()) {
             cout << "Vehicle " << vehicle.id << " : Path is not elementary" << endl;
        }
        // Get all the info we need to build a Route object
        double total_cost = vehicle.cost;
        double total_duration = 0;
        double total_travelling_time = 0;
        double total_waiting_time = 0;
        vector<int> id_sequence;
        vector<int> is_in_route(instance.nodes.size(), 0);
        vector<double> start_times(instance.nodes.size(), 0);
        
        // Keep track of the time ellapsed
        double current_time = 0;
        for (int i = 0; i < tour.size() - 1; i++) {
            int true_i = tour[i] == origin || tour[i] == destination ? vehicle.depot : vehicle.interventions[tour[i]];
            int true_j = tour[i + 1] == origin || tour[i + 1] == destination ? vehicle.depot : vehicle.interventions[tour[i + 1]];
            // Update the sequence of interventions
            id_sequence.push_back(true_i);
            // Update the is_in_route and start_times vectors
            is_in_route[true_i] = 1;
            start_times[true_i] = current_time;
            // Get the duration, and travel time and distance between the two interventions
            int duration = instance.nodes[true_i].duration;
            double travel_time = metric((instance.nodes[true_i]), (instance.nodes[true_j]), instance.time_matrix);
            double travel_distance = metric((instance.nodes[true_i]), (instance.nodes[true_j]), instance.distance_matrix);
            // Update the running total cost, duration of interventions and travel time
            total_cost += instance.cost_per_km * travel_distance;
            total_duration += duration;
            total_travelling_time += travel_time;
            
            // Update the current time
            double start_time_next = instance.nodes[true_j].start_window;
            double waiting_time = std::max(0.0, start_time_next - (current_time + duration + travel_time));
            total_waiting_time += waiting_time;
            current_time = std::max(current_time + duration + travel_time, start_time_next); 
        }
        // Add the checks related to the last intervention
        int true_last = vehicle.depot;
        id_sequence.push_back(true_last);
        is_in_route[true_last] = 1;
        start_times[true_last] = current_time;
        
        routes.push_back(Route(vehicle.id, total_cost, reduced_cost, total_duration, total_travelling_time, total_waiting_time, id_sequence, is_in_route, start_times));
    }

    
    return routes;

}