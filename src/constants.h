// In this file we define global constants needed in the problem
#pragma once

// Time constants
const int START_MORNING = 470;
const int END_MORNING = 720;
const int START_AFTERNOON = 810;
const int END_AFTERNOON = 1010;
const int LUNCH_BREAK = START_AFTERNOON - END_MORNING;
const int MID_DAY = END_MORNING - START_MORNING;
const int END_DAY = END_AFTERNOON - START_MORNING - LUNCH_BREAK;
// Long intervention threshold
const int LONG_INTERVENTION = 120;