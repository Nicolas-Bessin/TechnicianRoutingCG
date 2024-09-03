#include "master.h"

#include <algorithm>
#include <functional>
#include <cassert>

// Addition of two vectors
template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::plus<T>());
    return result;
}
// Multiplication of a vector by a scalar
template <typename T>
std::vector<T> operator*(double scalar, const std::vector<T>& a)
{
    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), 
                   std::back_inserter(result), 
                   [scalar](const T& x) { return scalar * x; });
    return result;
}

// Addition of two maps
template <typename K, typename V>
std::map<K, V> operator+(const std::map<K, V>& a, const std::map<K, V>& b)
{
    std::map<K, V> result;
    for (const auto& [key, value] : a) {
        result[key] = value;
    }
    for (const auto& [key, value] : b) {
        result[key] += value;
    }
    return result;
}

// Multiplication of a map by a scalar
template <typename K, typename V>
std::map<K, V> operator*(double scalar, const std::map<K, V>& a)
{
    std::map<K, V> result;
    for (const auto& [key, value] : a) {
        result[key] = scalar * value;
    }
    return result;
}




DualSolution operator+(const DualSolution &lhs, const DualSolution &rhs) {
    DualSolution result;
    result.alphas = lhs.alphas + rhs.alphas;
    result.betas = lhs.betas + rhs.betas;
    result.upper_bound_duals = lhs.upper_bound_duals + rhs.upper_bound_duals;
    result.lower_bound_duals = lhs.lower_bound_duals + rhs.lower_bound_duals;
    return result;
}

DualSolution operator*(double scalar, const DualSolution &rhs) {
    DualSolution result;
    result.alphas = scalar * rhs.alphas;
    result.betas = scalar * rhs.betas;
    result.upper_bound_duals = scalar * rhs.upper_bound_duals;
    result.lower_bound_duals = scalar * rhs.lower_bound_duals;
    return result;
}