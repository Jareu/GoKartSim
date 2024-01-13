#pragma once
#include <cstdint>
#include <memory>
#include "Transponder.h"
#include "Controller.h"

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
	void setTargetSpeed(double target_speed);
	double getTargetSpeed();
	void placeAtStartLine(uint8_t starting_position);
	const Controller* getController();
private:
	void handleFinishLine();
	const float DRIVER_FACTOR_SCALE = 1000.f;
	const float NOISE_SPEED_FACTOR = 0.25f;
	uint8_t kart_number_;
	double progress_;
	double lifetime_;
	double target_speed_;
	double driver_factor_;
	Universe& universe_;
	std::unique_ptr<Controller> controller_;
	std::unique_ptr <Transponder> transponder_;
};