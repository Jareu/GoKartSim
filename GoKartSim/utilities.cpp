#include <iomanip>
#include <stdlib.h>     /* srand, rand */
#include <algorithm>
#include <chrono>
#include <sstream>
#include "globals.h"
#include "Noise.h"
#include "utilities.h"

std::string utilities::getRandomHexString(uint32_t length)
{

	auto timestamp = std::chrono::seconds(std::time(NULL)).count();
	auto noise = std::make_shared<Noise>(timestamp);
	return getRandomHexString(length, noise);
}

std::string utilities::getRandomHexString(uint32_t length, const std::shared_ptr<Noise> noise)
{
	if (noise == nullptr)
	{
		throw std::invalid_argument("Noise object is nullptr.");
		return "";
	}

	std::stringstream hexString;

	for (int i = 0; i < length; i++) {
		uint16_t randomNibble = std::floor(16 * noise->getRandom() / static_cast<double>(RAND_MAX));
		hexString << std::uppercase << std::hex << randomNibble;
	}
	auto return_string = hexString.str();
	return return_string;
}

std::string utilities::getHexTimestamp()
{
	auto timestamp = std::chrono::milliseconds(std::time(NULL)).count();
	std::stringstream hexString;

	if (timestamp >= TIMESTAMP_MAX) {
		timestamp -= TIMESTAMP_MAX;
	};

	hexString << std::uppercase << std::setw(7) << std::hex << timestamp;
	return hexString.str();
}