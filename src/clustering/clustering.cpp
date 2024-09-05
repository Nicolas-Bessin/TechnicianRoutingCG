#include "clustering.h"

#include <map>


int hamming_distance(const Vehicle& vehicle1, const Vehicle& vehicle2){
    using std::map;

    map<int, int> interventions;
    // Store the interventions of each vehicle in a map
    for (int intervention : vehicle1.interventions){
        interventions[intervention] += 1;
    }
    for (int intervention : vehicle2.interventions){
        interventions[intervention] += 1;
    }
    // Those that have a value of 1 are only in one of the vehicles - they contribute 1 to the distance
    int distance = 0;
    for (auto [intervention, count] : interventions){
        if (count == 1){
            distance += 1;
        }
    }

    return distance;
}