// Definition of the various classes used to represent an instance of the problem
#pragma once
#include <map>
#include <vector>
#include <string>
#include <set>


struct Node {
    // id is a unique string identifier for the node
    std::string id;
    // node_id is the index of the node in the time and distance matrice
    int node_id;
    // Is this node an intervention ? If not, it is a warehouse
    bool is_intervention;
    // Number of vehicles that can perform the intervention (left empty for warehouses)
    int nb_vehicles;
    // Skills required to perform the intervention (left empty for warehouses) (counting the number of technicians needed with each skill)
    std::map<std::string, int> required_skills;
    // Duration of the intervention
    int duration;
    // Time window for the intervention
    int start_window;
    int end_window;
    // Is the intervention a long intervention?
    bool is_long;
    // Can the intervention be done both in the morning or in the afternoon?
    bool is_ambiguous;
    // Quantities of each ressource used by the intervention
    // The ressources help prevent using up all the time doing interventions (to leave flexibility in the planning)
    std::map<std::string, int> quantities;
    // Dual value associated with the intervention
    double alpha; 
    // Sparse constructor for warehouses
    Node(std::string id, int node_id){
        this->id = id;
        this->node_id = node_id;
        this->is_intervention = false;
        this->duration = 0;
    };
    // Full constructor for interventions
    Node(std::string id, int node_id, int duration, int start_window, int end_window, bool is_long, std::map<std::string, int> skills, std::map<std::string, int> quantities){
        this->id = id;
        this->node_id = node_id;
        this->duration = duration;
        this->start_window = start_window;
        this->end_window = end_window;
        this->is_long = is_long;
        this->required_skills = skills;
        this->quantities = quantities;
        this->nb_vehicles = 0;
        this->alpha = 0;
        this->is_intervention = true;
        // By default, the intervention is considered ambiguous
        this->is_ambiguous = true;
    }
};

int metric(const Node& node1, const Node& node2, std::vector<std::vector<int>> metric_matrix);

struct Technician {
    std::string id;
    std::set<std::string> skills;
    std::map<std::string, int> capacities;
    std::string operationnal_base;
    // Empty constructor
    Technician(){}
    // Constructor
    Technician(std::string id, std::set<std::string> skills, std::map<std::string, int> capacities, std::string operationnal_base){
        this->id = id;
        this->skills = skills;
        this->capacities = capacities;
        this->operationnal_base = operationnal_base;
    }
};

struct Vehicle {
    int id;
    // Counts the number of technicians that have each skill in the vehicle
    std::map<std::string, int> skills; 
    // List of indexes of interventions that the vehicle can perform
    // (indexes with respect to the nodes std::vector in the instance)
    std::vector<int> interventions;
    // Index of the depot in the nodes std::vector
    int depot;
    std::map<std::string, int> capacities;
    int start_window;
    int lunch_window;
    int end_window;
    double cost;
    // Constructor
    Vehicle(int id, std::map<std::string, int> skills, std::vector<int> interventions, int depot, std::map<std::string, int> capacities, int start_window, int lunch_window, int end_window, double cost){
        this->id = id;
        this->skills = skills;
        this->interventions = interventions;
        this->depot = depot;
        this->capacities = capacities;
        this->start_window = start_window;
        this->lunch_window = lunch_window;
        this->end_window = end_window;
        this->cost = cost;
    }
};

struct Instance {
    int number_interventions;
    int number_warehouses;
    int number_vehicles;
    double cost_per_km;
    double technician_cost;
    // Big M used in the objective function
    double M;
    // std::map from intervention / warehouse id to the corresponding index in the nodes std::vector
    std::map<std::string, int> node_id_to_index;
    // std::vector of interventions and warehouses
    std::vector<Node> nodes;
    // std::vector of vehicles
    std::vector<Vehicle> vehicles;
    // Different capacities considered
    std::vector<std::string> capacities_labels;
    // time between the different nodes
    // Indexes are the node ids
    std::vector<std::vector<int>> time_matrix;
    // distance between the different nodes
    // Indexes are the node ids
    std::vector<std::vector<int>> distance_matrix;

    // Constructor
    Instance(
        int number_interventions,
        int number_warehouses,
        int number_vehicles,
        double cost_per_km,
        double technician_cost, 
        double M,
        std::map<std::string, int> node_id_to_index,
        std::vector<Node> nodes,
        std::vector<Vehicle> vehicles,
        std::vector<std::string> capacities_labels, 
        std::vector<std::vector<int>> time_matrix,
        std::vector<std::vector<int>> distance_matrix)
    {
        this->number_interventions = number_interventions;
        this->number_warehouses = number_warehouses;
        this->number_vehicles = number_vehicles;
        this->cost_per_km = cost_per_km;
        this->technician_cost = technician_cost;
        this->M = M;
        this->node_id_to_index = node_id_to_index;
        this->nodes = nodes;
        this->vehicles = vehicles;
        this->capacities_labels = capacities_labels;
        this->time_matrix = time_matrix;
        this->distance_matrix = distance_matrix;
    }

};