#include "Characterisation.h"

PidData characterisation::characteriseController(Controller controller, double setpoint)
{
	PidData data;

	controller.initialise();
	controller.setSetPoint(setpoint);

	double time = 0.0;
	double last_error = 0.0;
	double dError = 0.0;

	do {
		double value = controller.update(CHARACTERISATION_TIMESTEP_);
		controller.setMeasurement(value);
		time += CHARACTERISATION_TIMESTEP_;

		// calculate error delta
		double error = controller.getError();
		dError = last_error - error;
		last_error = error;

		data.time_data.emplace_back(time);
		data.value_data.emplace_back(value);
		data.error_data.emplace_back(error);
	} while (abs(dError) > CHARACTERISATION_THRESHOLD_);

	return data;
}