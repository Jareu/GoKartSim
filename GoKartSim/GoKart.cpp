#include "GoKart.h"
#include "globals.h"
#include "Universe.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

GoKart::GoKart(Universe& universe, uint8_t kart_number, double progress) :
	universe_ { universe },
	progress_ { progress },
	kart_number_{ kart_number },
	lifetime_{ 0.0 },
	target_speed_{ 0.0 },
	controller_{ std::make_unique<Controller>() }
{
	transponder_ = std::make_unique <Transponder>(universe, *this);
	driver_factor_ = universe_.getNoise()->getRandom() * DRIVER_FACTOR_SCALE;
	std::cout << std::setprecision(3) << "driver factor: " << driver_factor_ << std::endl;
}

void GoKart::placeAtStartLine(uint8_t starting_position)
{
	progress_ = POSITION_SCALE * -PI * starting_position / static_cast<double>(std::numeric_limits<uint8_t>::max());
}

double GoKart::getProgress() const
{
	return progress_;
}

double GoKart::getTargetSpeed()
{
	return target_speed_;
}

void GoKart::setTargetSpeed(double target_speed)
{
	target_speed_ = target_speed;
}

void GoKart::advance(double delta_seconds)
{
	lifetime_ += delta_seconds;
	float noise_val = 1.f - (universe_.getNoise()->getNoise2D(lifetime_ / NOISE_SCALE, driver_factor_ / NOISE_SCALE) + 1.f) * 0.5f * NOISE_SPEED_FACTOR;
	double setpoint = noise_val * target_speed_;
	controller_->setSetPoint(setpoint);
	double speed = controller_->update(delta_seconds) * POSITION_SCALE;
	double new_progress = std::fmod(progress_ + (delta_seconds * speed), TWO_PI);

	if (new_progress < progress_) {
		handleFinishLine();
	}

	progress_ = new_progress;
}

void GoKart::handleFinishLine()
{
	std::string transponder_data = transponder_->getData();
	universe_.sendData(transponder_data);
}

uint8_t GoKart::getKartNumber() const
{
	return kart_number_;
}

const Controller* GoKart::getController()
{
	return controller_.get();
}
