#include "Transponder.h"
#include <iomanip>
#include <sstream>
#include "globals.h"
#include "utilities.h"
#include "Universe.h"

Transponder::Transponder(Universe& universe, GoKart& parent) :
	universe_ { universe },
	parent_ { parent }
{
	// $1482 2963501 AA00D10 ABCDEF 0
	transponderId_ = generateTransponderId();
}

std::string Transponder::generateTransponderId()
{
	std::stringstream idStream;
	uint32_t randomNumber = static_cast<uint32_t>(TRANSPONDER_ID_MAX * universe_.getNoise()->getRandom());
	idStream << std::uppercase << std::setfill('0') << std::setw(7) << std::hex << randomNumber;
	return idStream.str();
}

std::string Transponder::getData()
{
	std::stringstream data;
	auto timestamp = utilities::getHexTimestamp();
	auto randomHexString = utilities::getRandomHexString(6, universe_.getNoise());
	data << "$1482" << std::uppercase << transponderId_ << timestamp << randomHexString << "0";
	return data.str();
}