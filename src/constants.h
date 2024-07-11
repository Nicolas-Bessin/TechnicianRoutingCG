// In this file we define global constants needed in the problem
#pragma once


int START_MORNING = 470;
int END_MORNING = 720;
int START_AFTERNOON = 810;
int END_AFTERNOON = 1010;
int LUNCH_BREAK = START_AFTERNOON - END_MORNING;
int MID_DAY = END_MORNING - START_MORNING;
int END_DAY = END_AFTERNOON - START_MORNING - LUNCH_BREAK;