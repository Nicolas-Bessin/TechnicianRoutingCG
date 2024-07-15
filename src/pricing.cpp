#include "pricing.h"
#include <map>
#include <iostream>
#include "../pathwyse/core/solver.h"

// First, we want to define the network on which we will solve the problem
void find_best_route(const Instance& instance, const Vehicle& vehicle, const vector<double> &alphas, const double beta) {
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    // +1 for the "depature" warehouse and +1 for the "arrival" warehouse (even tough it is the same place)
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;
    // Get the number of ressources in the problem

    // Create a new instance of the Problem class
    Problem problem = Problem(
        "Constrained Vehicle Routing Problem",
        n_interventions_v + 2,
        origin,
        destination,
        0,
        true,
        false,
        true
    );
    // Initialize the problem
    problem.initProblem();

    // We use a default cost for the objective function
    DefaultCost* objective = new DefaultCost();
    objective->initData(false, n_interventions_v + 2);
    for (int i = 0; i < n_interventions_v; i++) {
        // Set the cost relative the node itself
        const Node* intervention_i = &(instance.nodes[vehicle.interventions[i]]);
        // Get the duration of the next intervention
        double duration = intervention_i->duration;
        // Set the cost of the intervention
        double node_cost = alphas.at(vehicle.interventions[i]) - instance.M * duration;
        objective->setNodeCost(i, node_cost);
        // Then set the arc costs for every arc leaving the intervention
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            // First step is adding an arc to the underlying network
            problem.setNetworkArc(i, j);
            // Get the intervention referenced by the index j

            const Node* intervention_j = &(instance.nodes[vehicle.interventions[j]]);
            // Get the distance between the two interventions
            double distance = metric(intervention_i, intervention_j, instance.distance_matrix);
            double arc_cost = instance.cost_per_km * distance;
            objective->setArcCost(i, j, arc_cost);  
        }
    }
    // Also setup the arcs between the warehouse and the interventions
    for (int i = 0; i < n_interventions_v; i++) {
        // get the intervention referenced by the index i
        const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
        // Outgoing arcs from the warehouse
        problem.setNetworkArc(origin, i);
        double distance_out = metric(&(instance.nodes[vehicle.depot]), intervention, instance.distance_matrix);
        // Set the arc cost, also adding the fixed cost of leaving the depot
        objective->setArcCost(origin, i, instance.cost_per_km * distance_out + vehicle.cost + beta);
        // Incoming arcs to the warehouse
        problem.setNetworkArc(i, destination);
        // Distance is probably the same as the outgoing distance but eh we never know
        double distance_in = metric(intervention, &(instance.nodes[vehicle.depot]), instance.distance_matrix);
        // When going from an intervention, we have to pay the cost of the intervention
        double duration = intervention->duration;
        double arc_cost = alphas.at(vehicle.interventions[i]) - instance.M * duration + instance.cost_per_km * distance_in;
        objective->setArcCost(i, destination, arc_cost);
    }

    // Set this resource as the objective function
    problem.initObjective(objective);

    // We then want to add the capacity ressources to the problem
    int nb_ressources = instance.capacities_labels.size();
    // Create a vector of ressources for the problem
    vector<Resource*> ressources;
    // For every label of capacity, we create a new capacity ressource
    for (int k = 0; k < nb_ressources; k++) {
        // Create a new capacity ressource
        Capacity* capacity = new Capacity();
        capacity->initData(false, n_interventions_v + 2);
        // Set the name of the ressource
        capacity->setName(instance.capacities_labels[k]);
        // Set its upper bound
        capacity->setUB(vehicle.capacities.at(instance.capacities_labels[k]));
        // Set the node consumptions for the ressource
        for (int i = 0; i < n_interventions_v; i++) {
            // Get the intervention referenced by the index i
            const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
            // Get the capacity consumption of the intervention
            int consumption = intervention->quantities.at(instance.capacities_labels[k]);
            // Set the capacity consumption of the intervention
            capacity->setNodeCost(i, consumption);
        }
        // Add it to the vector of ressources
        ressources.push_back(capacity);
    }
    // Also add the time window ressource to the problem
    TimeWindow* time_window = new TimeWindow();
    time_window->initData(false, n_interventions_v + 2);
    time_window->setName("Time Window");
    // For each node, add the UB and LB and node consumption
    for (int i = 0; i < n_interventions_v; i++) {
        // Get the intervention referenced by the index i
        const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
        // Get the time window of the intervention
        int start_window = intervention->start_window;
        int end_window = intervention->end_window;
        // Set the UB and LB of the time window
        time_window->setNodeBound(n_interventions_v + 2, i, start_window, end_window);
        // Set the node consumption of the time window
        time_window->setNodeCost(i, intervention->duration);
    }
    // Also set the time consumption on the arcs between the interventions
    for (int i = 0; i < n_interventions_v; i++) {
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            // Get the two interventions referenced by the indices i and j
            const Node* intervention_i = &(instance.nodes[vehicle.interventions[i]]);
            const Node* intervention_j = &(instance.nodes[vehicle.interventions[j]]);
            // Get the time it takes to go from intervention i to intervention j
            double travel_time = metric(intervention_i, intervention_j, instance.time_matrix);
            // Set the time consumption on the arc
            time_window->setArcCost(i, j, travel_time);
        }
    }
    // // And finally the time consumption on the arcs between the warehouse and the interventions
    for (int i = 0; i < n_interventions_v; i++) {
        // Get the intervention referenced by the index i
        const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
        // Outgoing arcs from the warehouse
        double travel_time_out = metric(&(instance.nodes[vehicle.depot]), intervention, instance.time_matrix);
        time_window->setArcCost(origin, i, travel_time_out);
        // Incoming arcs to the warehouse
        // Time is probably the same as the outgoing time but eh we never know
        double travel_time_in = metric(intervention, &(instance.nodes[vehicle.depot]), instance.time_matrix);
        time_window->setArcCost(i, destination, travel_time_in);
    }
    // Add the time window ressource to the vector of ressources
    time_window->init(origin, destination);
    ressources.push_back(time_window);

    // Set the ressources of the problem
    problem.setResources(ressources);
    problem.printProblem();

    // We now want to solve the problem
    Solver* solver = new Solver();
    solver->setCustomProblem(problem);
    solver->setupAlgorithms();
    solver->solve();
    solver->printBestSolution();
    // Get the solution we found
    Path path = solver->getBestSolution();
    // Extract the tour from the solution and convert it to a vector
    list<int> tour_list = path.getTour();
    vector<int> tour{tour_list.begin(), tour_list.end()};
    // Print the tour
    cout << "----------------------------------------" << endl;
    int tour_length = tour.size();
    int current_time = 0;
    for (int i = 0; i < tour_length - 1; i++) {
        //cout << tour[i] << " -> " << tour[i + 1] << endl;
        int true_i = tour[i] == origin || tour[i] == destination ? vehicle.depot : vehicle.interventions[tour[i]];
        int true_j = tour[i + 1] == origin || tour[i + 1] == destination ? vehicle.depot : vehicle.interventions[tour[i + 1]];
        cout << "Current time : " << current_time << endl;
        cout << "Tour step : " << true_i << " -> " << true_j << endl;
        // Print the duration of the intervention
        cout << "Duration : " << instance.nodes[true_i].duration << endl;
        // Print the time it takes to go from intervention i to intervention j
        cout << "Travel time : " << instance.time_matrix[true_i][true_j] << endl;
        // Print the time window of the intervention
        cout << "Time window : " << instance.nodes[true_j].start_window << " - " << instance.nodes[true_j].end_window << endl;
        // Update the current time
        current_time += instance.nodes[true_i].duration + instance.time_matrix[true_i][true_j];
        
    }
    
    return;


}