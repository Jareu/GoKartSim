#pragma once

#include <random>

extern constexpr double PI = 3.141592653589793238462643383279;
extern constexpr double TWO_PI = 6.283185307179586476925286766558;
extern constexpr double POSITION_SCALE = 10.0;
extern constexpr int SEED = 1234;

struct PidData
{
	std::vector<double> time_data;
	std::vector<double> value_data;
	std::vector<double> error_data;
};

constexpr double DEFAULT_KP = 0.6;
constexpr double DEFAULT_KI = 0.0;
constexpr double DEFAULT_KD = 0.0;
constexpr double DEFAULT_SPEED = 1.0;