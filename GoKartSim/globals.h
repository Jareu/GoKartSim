#pragma once

#include <random>

extern constexpr double PI = 3.141592653589793238462643383279;
extern constexpr double TWO_PI = 6.283185307179586476925286766558;
extern constexpr double POSITION_SCALE = 0.25;
extern constexpr double NOISE_SCALE = 100.0;
extern constexpr int SEED = 1234;
extern constexpr uint32_t TIMESTAMP_MAX = 268'435'456;
extern constexpr uint32_t TRANSPONDER_ID_MAX = 268'435'456;

struct PidData
{
	std::vector<double> time_data;
	std::vector<double> value_data;
	std::vector<double> error_data;
};

constexpr double DEFAULT_SPEED = 1.0;