// Definition of the various classes used to represent an instance of the problem
#pragma once
#include <map>
#include <vector>
#include <string>
#include <set>

using namespace std;


// Parent class to represent an intervention or a warehouse
class Node {
    public:
        // id is a unique string identifier for the node
        string id;
        // node_id is the index of the node in the time and distance matrice
        int node_id;
        // Number of vehicles that can perform the intervention (left empty for warehouses)
        int nb_vehicles;
        // Skills required to perform the intervention (left empty for warehouses)
        set<string> required_skills;
        // Empty constructor
        Node(){}
        // Destructor
        ~Node(){}
};


class Intervention : public Node {
    public:
        int duration;
        int start_window;
        int end_window;
        bool is_long;
        // Number of possible vehicles that can perform the intervention
        int nb_vehicles;
        // Quantities of each ressource used by the intervention
        // The ressources help prevent using up all the time doing interventions (to leave flexibility in the planning)
        map<string, int> quantities;
        // Dual value associated with the intervention
        double alpha; 
        // Constructor
        Intervention(string id, int node_id, int duration, int start_window, int end_window, bool is_long, set<string> skills, map<string, int> quantities){
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
        }
        // Destructor
        ~Intervention(){}
};

class Warehouse : public Node {
    public:
        // Empty constructor
        Warehouse(){}
        // Constructor
        Warehouse(string id, int node_id){
            this->id = id;
            this->node_id = node_id;
            this->required_skills = set<string>();
        }
        // Destructor
        ~Warehouse(){}
};

class Technician {
    public:
        string id;
        set<string> skills;
        map<string, int> capacities;
        string operationnal_base;
        // Empty constructor
        Technician(){}
        // Constructor
        Technician(string id, set<string> skills, map<string, int> capacities, string operationnal_base){
            this->id = id;
            this->skills = skills;
            this->capacities = capacities;
            this->operationnal_base = operationnal_base;
        }
        // Destructor
        ~Technician(){}
};

class Vehicle {
    public:
        int id;
        set<string> skills;
        vector<Node*> interventions;
        int depot;
        map<string, int> capacities;
        int start_window;
        int lunch_window;
        int end_window;
        double cost;
        // Constructor
        Vehicle(int id, set<string> skills, vector<Node*> interventions, int depot, map<string, int> capacities, int start_window, int lunch_window, int end_window, double cost){
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
        // Destructor
        ~Vehicle(){}
};

class Route {
    public:
        int id;
        double cost;
        int duration_interventions;
        Vehicle* vehicle;
        vector<Intervention*> interventions;
        map<int, int> start_times;
        // Constructor
        Route(int id, double cost, int duration_interventions, Vehicle* vehicle, vector<Intervention*> interventions, map<int, int> start_times){
            this->id = id;
            this->cost = cost;
            this->duration_interventions = duration_interventions;
            this->vehicle = vehicle;
            this->interventions = interventions;
            this->start_times = start_times;
        }
        // Destructor
        ~Route(){}
};

class Instance {
    public:
        int number_interventions;
        int number_warehouses;
        int number_vehicles;
        double cost_per_km;
        double technician_cost;
        // Map from intervention / warehouse id to the corresponding index in the nodes vector
        map<string, int> node_id_to_index;
        // Vector of interventions and warehouses
        vector<Node> nodes;
        // Vector of vehicles
        vector<Vehicle> vehicles;
        // Different capacities considered
        vector<string> capacities_labels;
        // time between the different nodes
        // Indexes are the node ids
        vector<vector<double>> time_matrix;
        // distance between the different nodes
        // Indexes are the node ids
        vector<vector<double>> distance_matrix;

        // Constructor
        Instance(
            int number_interventions,
            int number_warehouses,
            int number_vehicles,
            double cost_per_km,
            double technician_cost, 
            map<string, int> node_id_to_index,
            vector<Node> nodes,
            vector<Vehicle> vehicles,
            vector<string> capacities_labels, 
            vector<vector<double>> time_matrix,
            vector<vector<double>> distance_matrix)
        {
            this->number_interventions = number_interventions;
            this->number_warehouses = number_warehouses;
            this->number_vehicles = number_vehicles;
            this->cost_per_km = cost_per_km;
            this->technician_cost = technician_cost;
            this->node_id_to_index = node_id_to_index;
            this->nodes = nodes;
            this->vehicles = vehicles;
            this->capacities_labels = capacities_labels;
            this->time_matrix = time_matrix;
            this->distance_matrix = distance_matrix;
        }

        // Destructor
        ~Instance(){}

};