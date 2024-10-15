// In this file we define global constants needed in the problem
#pragma once

#include <array>

inline constexpr int START_MORNING = 470;
inline constexpr int END_MORNING = 720;
inline constexpr int START_AFTERNOON = 810;
inline constexpr int END_AFTERNOON = 1010;
inline constexpr int LUNCH_BREAK = 90;
inline constexpr int MID_DAY = 250;
inline constexpr int END_DAY = 450;
inline constexpr int LONG_INTERVENTION = 120;


// Define the groups of instances
inline const std::array<std::string, 5> SMALL_INSTANCES = {
    "agency1_19-01-2023_anonymized",
    "agency1_05-12-2022_anonymized",
    "agency1_09-12-2022_anonymized",
    "agency1_03-02-2023_anonymized",
    "agency1_23-11-2022_anonymized"
};
inline constexpr int SMALL_SIZE = 75;

inline const std::array<std::string, 5> MEDIUM_INSTANCES = {
    "agency1_09-12-2022_anonymized",
    "agency1_03-02-2023_anonymized",
    "agency1_23-11-2022_anonymized",
    "agency1_22-11-2022_anonymized",
    "agency1_17-01-2023_anonymized"
};
inline constexpr int MEDIUM_SIZE = 150;

inline const std::array<std::string, 5> LARGE_INSTANCES = {
    "agency1_23-11-2022_anonymized",
    "agency1_22-11-2022_anonymized",
    "agency1_17-01-2023_anonymized",
    "agency1_18-01-2023_anonymized",
    "agency1_08-02-2023_anonymized"
};
inline constexpr int LARGE_SIZE = 200;

