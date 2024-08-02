#include "solution_converter.h"


std::vector<Route> compact_solution_to_routes(const Instance& instance, const CompactSolution<int>& compact_solution) {
    using std::vector;
    using std::cout, std::endl;

    int n_nodes = instance.nodes.size();
    int n_intervention = instance.number_interventions;
    int n_vehicles = instance.number_vehicles;

    // We will build a vector of routes from the compact solution
    vector<Route> routes;

    // For each vehicle, we will build a route if it is used
    for (int v = 0; v < n_vehicles; v++) {
        if (compact_solution.y[v] == 0) continue;
        //cout << "Vehicle " << v << " is used" << endl;
        const Vehicle& vehicle = instance.vehicles[v];
        // Initialize the route components
        double total_cost = vehicle.cost;
        double total_duration = 0;
        double total_travelling_time = 0;
        double total_waiting_time = 0;
        vector<int> sequence;
        vector<int> is_in_route = vector<int>(n_nodes, 0);
        vector<double> start_times = vector<double>(n_nodes, 0);
        // Go through the route
        bool reached_depot = false;
        int current_node = vehicle.depot;
        int current_time = 0;

        while (!reached_depot) {
            // Update the info relative to the current node
            //cout << "Current node : " << current_node << endl;
            const Node& node = instance.nodes[current_node];
            sequence.push_back(current_node);
            is_in_route[current_node] = 1;
            if (current_node == vehicle.depot) {
                start_times[current_node] = 0;
            } else {
                start_times[current_node] = compact_solution.u[current_node];
            }
            // Update the counters : waiting time is max between current start time and time elapsed until now
            total_waiting_time += std::max(0.0, start_times[current_node] - current_time);
            current_time = start_times[current_node] + node.duration;
            total_duration += node.duration;
            // Find the next node
            int next_node = -1;
            for (int i = 0; i < n_nodes; i++) {
                if (compact_solution.x[current_node][i][v] == 1) {
                    next_node = i;
                    break;
                }
            }
            if (next_node == -1) {
                std::cerr << "Error : no next node found" << std::endl;
                break;
            }
            // If the next node is already in the route and is not the depot, we have a problem
            if (is_in_route[next_node] == 1 && next_node != vehicle.depot) {
                std::cerr << "Error : next node is already in the route" << " / Next node : " << next_node << std::endl;
                break;
            }
            const Node& next = instance.nodes[next_node];
            // Update the travelling time & cost
            //cout << "Current number of nodes : " << sequence.size() << endl;
            //cout << "Current node : " << current_node << " / Next node : " << next_node << endl;
            int travelling_time = instance.time_matrix.at(node.node_id).at(next.node_id);
            total_travelling_time += travelling_time;
            int distance = instance.distance_matrix.at(node.node_id).at(next.node_id);
            total_cost += distance * instance.cost_per_km;

            current_time += travelling_time;
            // Check if we reached the depot
            if (next_node == vehicle.depot) {
                reached_depot = true;
            } else {
                current_node = next_node;
            }
        }
        //cout << "------" << endl;
        // Add the depot to the sequence
        sequence.push_back(vehicle.depot);

        // Create the route
        Route route = Route(v, total_cost, 0.0, total_duration, total_travelling_time, total_waiting_time, sequence, is_in_route, start_times);
        routes.push_back(route);
    }

    return routes;
}


CompactSolution<double> to_compact_solution(const MasterSolution& master_solution, const std::vector<Route>& routes, const Instance& instance) {
    using std::vector;
    using std::cout, std::endl;

    int n_nodes = instance.nodes.size();
    int n_intervention = instance.number_interventions;
    int n_vehicles = instance.number_vehicles;

    // Initialize the compact solution - the vectors are already initialized to 0
    CompactSolution<double> compact_solution(n_nodes, n_intervention, n_vehicles);


    // Enumerate through all routes to update the compact solution
    for (int r = 0; r < routes.size(); r++) {
        // Skip the routes that are not used
        if (master_solution.coefficients[r] == 0) continue;
        const Route& route = routes[r];
        // Update the y variable - the vehicle v is used x_r times (x_r is fractional)
        compact_solution.y[route.vehicle_id] += master_solution.coefficients[r];
        // We don't care about the u and z variables for now - they are onyl relevant for defining an admissible solution
        // But do not count in the cost of the solution
        // Update the x variables
        for (int i = 0; i < route.id_sequence.size() - 1; i++) {
            int current_node = route.id_sequence[i];
            int next_node = route.id_sequence[i + 1];
            //cout << "Current node : " << current_node << " / Next node : " << next_node << endl;
            compact_solution.x[current_node][next_node][route.vehicle_id] += master_solution.coefficients[r];
        }
    }

    return compact_solution;
}

CompactSolution<int> to_compact_solution(const IntegerSolution& integer_solution, const std::vector<Route>& routes, const Instance& instance) {
    using std::vector;
    using std::cout, std::endl;

    int n_nodes = instance.nodes.size();
    int n_intervention = instance.number_interventions;
    int n_vehicles = instance.number_vehicles;

    // Initialize the compact solution - the vectors are already initialized to 0
    CompactSolution<int> compact_solution(n_nodes, n_intervention, n_vehicles);

    // Enumerate through all routes to update the compact solution
    for (int r = 0; r < routes.size(); r++) {
        // Skip the routes that are not used
        if (integer_solution.coefficients[r] == 0) continue;
        const Route& route = routes[r];
        // Update the y variable - the vehicle v is used x_r times (x_r is fractional)
        compact_solution.y[route.vehicle_id] += integer_solution.coefficients[r];
        // We don't care about the u and z variables for now - they are onyl relevant for defining an admissible solution
        // But do not count in the cost of the solution
        // Update the x variables
        for (int i = 0; i < route.id_sequence.size() - 1; i++) {
            int current_node = route.id_sequence[i];
            int next_node = route.id_sequence[i + 1];
            //cout << "Current node : " << current_node << " / Next node : " << next_node << endl;
            compact_solution.x[current_node][next_node][route.vehicle_id] += integer_solution.coefficients[r];
        }
    }

    return compact_solution;
}