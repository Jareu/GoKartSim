#pragma once
#include <cstdint>

class Universe;

class GoKart
{
public:
	GoKart() = delete;
	GoKart(Universe& universe, uint8_t kart_number, double progress);
	~GoKart() = default;

	void advance(double delta_seconds);
	double getProgress() const;
	uint8_t getKartNumber() const;
	void setSpeed(double speed);
	void placeAtStartLine(uint8_t starting_position);
private:
	uint8_t kart_number_;
	double progress_;
	double lifetime_;
	double speed_;
	double driver_factor_;
	Universe& universe_;
};