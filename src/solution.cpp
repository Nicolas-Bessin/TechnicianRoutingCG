#include "solution.h"
#include <math.h>
#include <algorithm>

using std::min;

// Check wether two consecutive master solutions are equal (i.e whether their non zero coefficients are the same)
bool operator==(const MasterSolution& lhs, const MasterSolution& rhs){
    // Check the objective value
    if (fabs(lhs.objective_value - rhs.objective_value) > 0.0001){
        return false;
    }
    // Check the length of the coefficients
    // Check the coefficients themselves
    int size = min(lhs.coefficients.size(), rhs.coefficients.size());
    for (int i = 0; i < size; i++){
        if (lhs.coefficients[i] != rhs.coefficients[i]){
            return false;
        }
    }
    return true;
}

// Checks if two routes are equal
// That is, if they have :
// - the same vehicle_id
// - the same sequence vector
// - the same start times
bool operator==(const Route& lhs, const Route& rhs){
    // Check the vehicle_id
    if (lhs.vehicle_id != rhs.vehicle_id){
        return false;
    }
    //Check the length of the id_sequence
    if (lhs.id_sequence.size() != rhs.id_sequence.size()){
        return false;
    }
    // Check the sequences themselves
    for (int i = 0; i < lhs.id_sequence.size(); i++){
        if (lhs.id_sequence[i] != rhs.id_sequence[i]){
            return false;
        }
    }
    // Check the start times
    for (int i = 0; i < lhs.start_times.size(); i++){
        if (fabs(lhs.start_times[i] - rhs.start_times[i]) > 0.0001){
            return false;
        }
    }

    return true;
}


bool operator<(const Route& lhs, const Route& rhs){
    // Check the vehicle_id
    if (lhs.vehicle_id < rhs.vehicle_id){
        return true;
    }
    if (lhs.vehicle_id > rhs.vehicle_id){
        return false;
    }
    //Check the length of the id_sequence
    if (lhs.id_sequence.size() < rhs.id_sequence.size()){
        return true;
    }
    if (lhs.id_sequence.size() > rhs.id_sequence.size()){
        return false;
    }
    // Check the sequences themselves
    for (int i = 0; i < lhs.id_sequence.size(); i++){
        if (lhs.id_sequence[i] < rhs.id_sequence[i]){
            return true;
        }
        if (lhs.id_sequence[i] > rhs.id_sequence[i]){
            return false;
        }
    }
    // Check the start times
    for (int i = 0; i < lhs.start_times.size(); i++){
        if (lhs.start_times[i] < rhs.start_times[i]){
            return true;
        }
        if (lhs.start_times[i] > rhs.start_times[i]){
            return false;
        }
    }
    // If we reach this point, the two routes are equal
    return false;
}