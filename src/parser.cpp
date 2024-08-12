#include "parser.h"
#include "constants.h"
#include "../nlohmann/json.hpp"

#include <iostream>
#include <bits/stdc++.h>
// For the gcd function
#include <algorithm>

using json = nlohmann::json;
using std::vector, std::string, std::map, std::set;
using std::ifstream, std::cout, std::endl;
using std::min, std::max, std::__gcd;

// Convert a time into a relative time
// 0 is the beginning of the work day
// Lunch break not included in the work time measurement
int convert_to_relative_time(int end_time){
    if (end_time < START_MORNING){
        return 0;
    } else if (end_time < END_MORNING){
        return end_time - START_MORNING;
    } else if (end_time < START_AFTERNOON){
        return MID_DAY;
    } else if (end_time < END_AFTERNOON){
        return end_time - START_MORNING - LUNCH_BREAK;
    } else {
        return END_DAY;
    }
}

// Parse a JSON object into an Node object
Node parse_intervention(json data){
    string id = data.at("id");
    int node_id = data.at("node_id");
    int duration = data.at("duration");
    int start_window = convert_to_relative_time(data.at("start_window"));
    int end_window = convert_to_relative_time(data.at("end_window"));
    bool is_long = duration >= LONG_INTERVENTION;
    map<string, int> skills = map<string, int>();
    // data.at("skills") is a list of list of skills (i.e the skills needed by each technician)
    for (const auto &skills_per_tech : data.at("skills")){
        for (const auto &skill : skills_per_tech){
            // If the skill is not in the map, it's initialized to 0 using the [] access operator
            skills[skill] += 1;
        }
    }
    map<string, int> quantities = data.at("quantities");
    return Node(id, node_id, duration, start_window, end_window, is_long, quantities, skills);
}


// Parse a JSON object into a Node object
Node parse_warehouse(json data){
    int node_id = data.at("node_id");
    string ope_base = data.at("ope_base");
    return Node(ope_base, node_id);
}


// Parse a technician JSON object into a Technician object
Technician parse_technician(json data){
    string id = data.at("id");
    set<string> skills = set<string>();
    for (const auto &skill : data.at("skills")){
        skills.insert(skill);
    }
    map<string, int> capacities = data.at("capacities");
    string operationnal_base = data.at("ope_base");
    return Technician(id, operationnal_base, skills, capacities);
}



// Parse a JSON file to return a Instance object
Instance parse_file(string filename, bool verbose){
    if (verbose){
        cout << "Parsing file " << filename << endl;
    }

    // Read the JSON file
    ifstream f(filename);
    json data = json::parse(f);

    // Get the main constants
    json constants = data.at("const_manager");
    double cost_per_km = constants.at("km_cost");
    double tech_cost = constants.at("tech_cost");
    int number_ressources = constants.at("capacities_size");
    vector<string> ressources = constants.at("capacities_labels");
    // Remove "JOU", "MA" and "AP" from those labels
    vector<string> removed_ressources = vector<string>{"JOU", "MA", "AP"};
    for (const auto &element : removed_ressources){
        auto pos = find(ressources.begin(), ressources.end(), element);
        if (pos != ressources.end()){
            ressources.erase(pos);
        }
    };

    // Print the main constants
    if (verbose){
        cout << "Number of ressources: " << number_ressources << endl;
        cout << "Ressources: ";
        for (const auto &ressource : ressources){
            cout << ressource << " ";
        }
        cout << endl;
        cout << "Cost per km: " << cost_per_km << endl;
        cout << "Technician cost: " << tech_cost << endl;
    }


    // Get the matrices of distance and time
    json loc_manager = data.at("loc_manager");
    vector<vector<int>> distance_matrix_raw = loc_manager.at("matrix").at("distance");
    vector<vector<int>> time_matrix_raw = loc_manager.at("matrix").at("time");

    // For the interventions and warehouse, we wil put them into a single vector
    // Also build a map from intervention / warehouse id to index in the vector
    vector<Node> nodes = vector<Node>();
    map<string, int> node_id_to_index = map<string, int>();

    // Get the interventions
    vector<json> interventions_data = data.at("step_manager").at("interventions");
    for (int i = 0; i < interventions_data.size(); i++){
        Node intervention = parse_intervention(interventions_data[i]);
        nodes.push_back(intervention);
        node_id_to_index[intervention.id] = i;
    }

    // Print the number of interventions
    int nb_interventions = nodes.size();
    if (verbose){
        cout << "Number of interventions: " << nb_interventions << endl;
    }

    // Get the warehouses and add them to the nodes
    vector<json> warehouses_data = data.at("step_manager").at("warehouses");
    for (int i = 0; i < warehouses_data.size(); i++){
        Node warehouse = parse_warehouse(warehouses_data[i]);
        // Time window for the warehouse is the whole day
        warehouse.start_window = 0;
        warehouse.end_window = END_DAY;
        nodes.push_back(warehouse);
        node_id_to_index[warehouse.id] = i + nb_interventions;
    }

    // Print the number of warehouses and number of nodes
    int nb_warehouses = warehouses_data.size();
    if (verbose){
        cout << "Number of warehouses: " << nb_warehouses << endl;
        cout << "Number of nodes: " << nodes.size() << endl;
    }

    // Build the time and distance matrix for the nodes :
    // time_matrix[i][j] is the time to go from node i to node j
    vector<vector<int>> time_matrix = vector<vector<int>>(nodes.size(), vector<int>(nodes.size(), 0));
    vector<vector<int>> distance_matrix = vector<vector<int>>(nodes.size(), vector<int>(nodes.size(), 0));
    for (int i = 0; i < nodes.size(); i++){
        for (int j = 0; j < nodes.size(); j++){
            // The distance and time matrices are indexed by the node_id
            int node1_id = nodes[i].node_id;
            int node2_id = nodes[j].node_id;
            distance_matrix[i][j] = distance_matrix_raw.at(node1_id).at(node2_id);
            time_matrix[i][j] = time_matrix_raw.at(node1_id).at(node2_id);
        }
    }


    // Before building the teams, we put all the technicians into a dictionary for easy access
    vector<json> technicians_data = data.at("tech_manager").at("technicians");
    map<string, Technician> technicians = map<string, Technician>();
    for (int i = 0; i < technicians_data.size(); i++){
        Technician tech = parse_technician(technicians_data[i]);
        technicians[tech.id] = tech;
    }

    // Print the number of technicians
    if (verbose){
        cout << "Number of technicians: " << technicians.size() << endl;
    }

    // Teams are groups of technicians
    // Initially, we have fixed teams
    vector<vector<string>> tech_id_per_team = data.at("tech_manager").at("teams").at("fixed_teams");
    // Also keep track of the technicians already in a team
    set<string> tech_with_team = set<string>();
    for (const auto &team_ids : tech_id_per_team){
        tech_with_team.insert(team_ids.begin(), team_ids.end());
    }
    // We then add every technician not into a fixed team as a team of one
    for (const auto &[tech_id, tech] : technicians){
        if (tech_with_team.contains(tech_id) == false){
            tech_id_per_team.push_back(vector<string>{tech_id});
        }
    }
    
    // Print the number of teams
    if (verbose){
        cout << "Number of teams: " << tech_id_per_team.size() << endl;
    }

    // We now construct one vehicle per team
    vector<Vehicle> vehicles = vector<Vehicle>{};
    for (int v = 0; v < tech_id_per_team.size(); v++){
        // Collect the technicians in the team
        vector<string> team_ids = tech_id_per_team[v];
        // Build the map of available skills for the vehicle
        map<string, int> skills = map<string, int>();
        for (const string & id : team_ids){
            for (const string &skill : technicians[id].skills){
                // If accessing the skill for the first time, it is initialized to 0 by the [] operator
                skills[skill] += 1;
            }
        }
        // Check that every technician in the team has the same operationnal base
        string operationnal_base = technicians[team_ids[0]].operationnal_base;
        for (const string& id : team_ids){
            if (technicians[id].operationnal_base != operationnal_base){
                cout << "Warning : technicians in the same team must have the same operationnal base - team " << v << endl;
                cout << "Technician " << id << " has operationnal base " << technicians[id].operationnal_base << endl;
                cout << "Technician " << team_ids[0] << " has operationnal base " << technicians[team_ids[0]].operationnal_base << endl;
                //operationnal_base = "ope_base5";
                //cout << "HARD-CODING the operationnal base to " << operationnal_base << endl;
            }
        }
        // Get the depot node
        int depot = node_id_to_index[operationnal_base];
        // Build the capacities of the vehicles as the min of the capacities of each technician for each ressource
        map<string, int> capacities = map<string, int>();
        for (const string & label : ressources){
            int min_capacity = technicians[team_ids[0]].capacities[label];
            for (const string &id : team_ids){
                min_capacity = min(min_capacity, technicians[id].capacities[label]);
            }
            capacities[label] = min_capacity;
        }
        // Vehicle cost is the sum of the cost of each technician in the team
        double vehicle_cost = tech_cost * team_ids.size();
        vehicles.push_back(Vehicle(v, team_ids, skills, vector<int>(), map<int, int>(), depot, capacities, vehicle_cost));
    }

    // Print the number of vehicles
    int nb_vehicles = vehicles.size();
    if (verbose){
        cout << "Number of vehicles: " << nb_vehicles << endl;
    }

    // We now build a cross reference matrix between interventions and vehicles : has_skill[i][j] is true if vehicle j has the skills to do intervention i
    vector<vector<bool>> has_skill = vector<vector<bool>>(nb_interventions, vector<bool>(nb_vehicles, false));
    for (int i = 0; i < nb_interventions; i++){
        for (int v = 0; v < nb_vehicles; v++){
            // We must ensure that the vehicle has enough technicians with each skill to do the intervention
            bool can_do = can_do_intervention(nodes[i], vehicles[v]);
            has_skill[i][v] = can_do;
        }
    }

    // And we can add references to the interventions that each vehicle can do
    for (int v = 0; v < nb_vehicles; v++){
        for (int i = 0; i < nb_interventions; i++){
            if (has_skill[i][v]){
                vehicles[v].interventions.push_back(i);
                vehicles[v].reverse_interventions[i] = vehicles[v].interventions.size() - 1;
            }
        }
    }

    // Finally, compute the big M for the objective function
    // M is computed using : M = (END_DAY - min(durations .> 0)) * maxspeed * cost_per_km / gcd(durations)
    // We first put all the durations in a vector
    vector<int> durations = vector<int>();
    // Enumerate through the nodes
    for (int i = 0; i < nb_interventions; i++){
        // Only keep the durations that are greater than 0
        if (nodes[i].duration > 0){
            durations.push_back(nodes[i].duration);
        }
    }
    int min_duration = *min_element(durations.begin(), durations.end());
    // We now compute the gcd of the durations
    int gcd_durations = durations[0];
    for (int i = 1; i < durations.size(); i++){
        gcd_durations = __gcd(gcd_durations, durations[i]);
    }
    // We finally compute the maximum speed using the ditance and time matrices
    // Only among the pairs of node that we parsed previously
    double max_speed = 0;
    std::pair<int, int> max_speed_pair = std::make_pair(0, 0);
    for (int i = 0; i < nodes.size(); i++){
        for (int j = 0; j < nodes.size(); j++){
            if (time_matrix[i][j] > 0){
                double speed = distance_matrix[i][j] / (double) time_matrix[i][j];
                if (speed > max_speed){
                    max_speed = speed;
                    max_speed_pair = std::make_pair(i, j);
                }
            }
        }
    }

    double M = (END_DAY - min_duration) * max_speed * cost_per_km / gcd_durations;

    if (verbose){
        // Print the values of the constants
        cout << "GCD is " << gcd_durations << endl;
        cout << "Max speed is " << max_speed << endl;
        cout << "Max speed is between " << max_speed_pair.first << " and " << max_speed_pair.second << endl;
        // Print the value of M
        cout << "Big M: " << M << endl;
    }

    // We can now build the instance object
    return Instance(
        nb_interventions,
        nb_warehouses,
        nb_vehicles,
        cost_per_km,
        tech_cost,
        M,
        nodes,
        vehicles,
        ressources,
        time_matrix,
        distance_matrix
    );    

}
    
