#pragma once
#include "globals.h"
#include "Controller.h"

namespace characterisation {
	constexpr double CHARACTERISATION_TIMESTEP_ = 0.01;
	constexpr double CHARACTERISATION_THRESHOLD_ = 0.00001;

	PidData characteriseController(Controller controller, double setpoint);
}