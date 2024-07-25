#include "pricing.h"
#include "constants.h"
#include "lunch.h"
#include "time_window_lunch.h"
#include "../pathwyse/core/solver.h"

#include <map>
#include <iostream>
#include <memory>


using std::cout, std::endl;
using std::vector, std::list, std::string;
using std::unique_ptr;


unique_ptr<Problem> create_pricing_instance(const Instance& instance, const Vehicle& vehicle, int scale_factor) {
    // We scale every cost up by the scale factor
    // in hopes of mitigating the issue of the costs & objctive values only being integers in pathwyse
    
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    // +1 for the "depature" warehouse and +1 for the "arrival" warehouse (even tough it is the same place)
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;
    // Get the number of resources in the problem

    // Create a new instance of the Problem class
    Problem* problem = new Problem(
        std::to_string(vehicle.id),
        n_interventions_v + 2,
        origin,
        destination,
        0,
        false,
        false,
        true
    );
    // Initialize the problem
    problem->initProblem();


    // Initialize the objective function
    DefaultCost* objective = new DefaultCost();
    objective->initData(false, n_interventions_v + 2);

    // Set the costs of the arcs
    for (int i = 0; i < n_interventions_v; i++) {
        const Node& intervention_i = (instance.nodes[vehicle.interventions[i]]);
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            // First step is adding an arc to the underlying network
            problem->setNetworkArc(i, j);
            // Get the intervention referenced by the index j
            const Node& intervention_j = instance.nodes[vehicle.interventions[j]];
            // Get the distance between the two interventions
            double distance = metric(intervention_i, intervention_j, instance.distance_matrix);
            double arc_cost = instance.cost_per_km * distance;
            objective->setArcCost(i, j, arc_cost * scale_factor);
        }
    }
    // Also setup the arcs between the warehouse and the interventions
    for (int i = 0; i < n_interventions_v; i++) {
        // get the intervention referenced by the index i
        const Node& intervention = (instance.nodes[vehicle.interventions[i]]);
        // Outgoing arcs from the warehouse
        problem->setNetworkArc(origin, i);
        double distance_out = metric((instance.nodes[vehicle.depot]), intervention, instance.distance_matrix);
        // Set the arc cost, also adding the fixed cost of the vehicle (but not beta)
        objective->setArcCost(origin, i, (instance.cost_per_km * distance_out + vehicle.cost) * scale_factor);
        // Incoming arcs to the warehouse
        problem->setNetworkArc(i, destination);
        // Distance is probably the same as the outgoing distance but eh we never know
        double distance_in = metric(intervention, (instance.nodes[vehicle.depot]), instance.distance_matrix);
        // Here we only consider the cost of travelng, the node costs have already been set
        objective->setArcCost(i, destination, (instance.cost_per_km * distance_in) * scale_factor);
    }

    // Set this resource as the objective function
    problem->setObjective(objective);

    // Create a vector of resources for the problem
    vector<Resource<int>*> resources;

    // Add the time window ressource to the problem as the critical ressource
    CustomTimeWindow* time_window = new CustomTimeWindow(n_interventions_v + 2);
    time_window->initData(false, n_interventions_v + 2);
    time_window->setName("Time Window + Lunch");
    // For each node, add the UB and LB and node consumption
    for (int i = 0; i < n_interventions_v; i++) {
        const Node& intervention = (instance.nodes[vehicle.interventions[i]]);
        // Get the time window of the intervention
        int start_window = intervention.start_window;
        int end_window = intervention.end_window - intervention.duration;
        // Set the time window of the intervention
        time_window->setNodeBound(n_interventions_v + 2, i, start_window, end_window);
        // Set the node consumption of the time window
        time_window->setNodeCost(i, intervention.duration);
        // Sets the lunch break constraint
        time_window->setLunchConstraint(i, intervention.is_ambiguous);
        // Also set the time consumption on the arcs between the interventions
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            // Get the two interventions referenced by the indices i and j
            const Node& intervention_j = (instance.nodes[vehicle.interventions[j]]);
            // Get the time it takes to go from intervention i to intervention j
            double travel_time = metric(intervention, intervention_j, instance.time_matrix);
            // Set the time consumption on the arc
            time_window->setArcCost(i, j, travel_time);
        }

        // Outgoing arcs from the warehouse
        double travel_time = metric((instance.nodes[vehicle.depot]), intervention, instance.time_matrix);
        time_window->setArcCost(origin, i, travel_time);
        time_window->setArcCost(i, destination, travel_time);
    }
    // The time windows on the warehouse :
    time_window->setNodeBound(n_interventions_v + 2, origin, 0, END_DAY);
    time_window->setNodeBound(n_interventions_v + 2, destination, 0, END_DAY);
   
    // Add the time window ressource to the vector of resources
    time_window->init(origin, destination);
    resources.push_back(time_window);

    // We then want to add the capacity resources to the problem
    int nb_capacities = instance.capacities_labels.size();
    // For every label of capacity, we create a new capacity ressource
    for (const string & label : instance.capacities_labels) {
        // Create a new capacity ressource
        Capacity* capacity = new Capacity();
        capacity->initData(false, n_interventions_v + 2);
        // Set the name of the ressource
        capacity->setName(label);
        // Set its upper bound
        capacity->setUB(vehicle.capacities.at(label) + 1);
        // Set the node consumptions for the ressource
        for (int i = 0; i < n_interventions_v; i++) {
            // Get the intervention referenced by the index i
            const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
            // Get the capacity consumption of the intervention
            int consumption = intervention->quantities.at(label);
            // Set the capacity consumption of the intervention
            capacity->setNodeCost(i, consumption);
        }
        // Add it to the vector of resources
        resources.push_back(capacity);
    }

    // Set the resources of the problem
    problem->setResources(resources);
    //problem->printProblem();

    return unique_ptr<Problem>(problem);
}



void update_pricing_instance(unique_ptr<Problem> & pricing_problem, const vector<double>& alphas, double beta,
        const Instance& instance, const Vehicle& vehicle, int scale_factor) {
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;
    // Get the objective function of the problem
    DefaultCost* objective = (DefaultCost*) pricing_problem->getObj();
    // Put in the node costs : alpha_i - M * duration_i
    for (int i = 0; i < n_interventions_v; i++) {
        int true_i = vehicle.interventions[i];
        double node_cost = alphas[true_i] - instance.M * instance.nodes[true_i].duration;
        objective->setNodeCost(i, node_cost * scale_factor);
    }
    // Put in the constant part (only beta in the pricing problem, the cost of the vehicle is already accounted for on the arcs)
    objective->setNodeCost(origin, beta * scale_factor);
    // Re-set the objective function
    pricing_problem->setObjective(objective);
    return;
}


vector<Route> solve_pricing_problem(unique_ptr<Problem> & problem, int pool_size, const Instance& instance, const Vehicle& vehicle) {
    // Get the origin and destination of the problem
    int origin = problem->getOrigin();
    int destination = problem->getDestination();
    // We now want to solve the problem
    Solver* solver = new Solver("../pathwyse.set");
    solver->setCustomProblem(*problem);
    solver->setupAlgorithms();
    solver->solve();
    // If the problem is indeterminate (time limit reached, we return an empty vector)
    if (solver->getProblem()->getStatus() == PROBLEM_INDETERMINATE) {
        cout << "Time limit reached for vehicle " << vehicle.id << endl;
        return vector<Route>();
    }
    //solver->printBestSolution();
    // Get the solution we found
    vector<Path> solutions = solver->getBestSolutions(pool_size);
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
            int true_i = i == 0 ? vehicle.depot : vehicle.interventions[tour[i]];
            int true_j = i+1 == tour.size()-1 ? vehicle.depot : vehicle.interventions[tour[i + 1]];
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

