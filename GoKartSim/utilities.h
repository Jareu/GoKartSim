#pragma once
#include <string>
#include "Noise.h"

namespace utilities
{
	std::string getRandomHexString(uint32_t length);
	std::string getRandomHexString(uint32_t length, const std::shared_ptr<Noise> noise);
	std::string getHexTimestamp();
}
