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
	const float DRIVER_FACTOR_SCALE = 1000.f;
	const float NOISE_SPEED_FACTOR = 0.25f;
	uint8_t kart_number_;
	double progress_;
	double lifetime_;
	double speed_;
	double driver_factor_;
	Universe& universe_;
};