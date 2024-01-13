#pragma once
#include <string>

// Head  TransponderID Timestamp ? Zero
// $1482 ###### $$$$$$$ ?????? 0
class GoKart;
class Universe;

class Transponder
{
public:
	Transponder() = delete;
	Transponder(Universe& universe, GoKart& parent);
	std::string getData();
	std::string generateTransponderId();
private:
	Universe& universe_;
	GoKart& parent_;
	std::string transponderId_;
};