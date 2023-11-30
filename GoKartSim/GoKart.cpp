#include "GoKart.h"
#include "globals.h"
#include "Universe.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

GoKart::GoKart(Universe& universe, uint8_t kart_number, double progress) :
	universe_ {universe},
	progress_ {progress},
	kart_number_{ kart_number },
	lifetime_{ 0.0 },
	speed_{ 0.0 }
{
	driver_factor_ = universe_.getNoise()->getRandom();
	std::cout << std::setprecision(3) << "driver factor: " << driver_factor_ << std::endl;
}

void GoKart::placeAtStartLine(uint8_t starting_position)
{
	progress_ = POSITION_SCALE * PI * starting_position / static_cast<double>(std::numeric_limits<uint8_t>::max());
}

double GoKart::getProgress() const
{
	return progress_;
}

void GoKart::setSpeed(double speed)
{
	speed_ = speed;
}

void GoKart::advance(double delta_seconds)
{
	lifetime_ += delta_seconds;
	float noise_val = universe_.getNoise()->getNoise2D(lifetime_, driver_factor_);
	progress_ = std::fmod(progress_ + (delta_seconds * speed_) * noise_val, PI);
}

uint8_t GoKart::getKartNumber() const
{
	return kart_number_;
}