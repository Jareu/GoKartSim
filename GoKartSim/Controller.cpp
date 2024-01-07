#include "Controller.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

Controller::Controller() : Controller(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN, DEFAULT_MAX, DEFAULT_TAU)
{}

Controller::Controller(double kP, double kI, double kD, double outputLimitMin, double outputLimitMax, double tau) :
	kP_{ kP },
	kI_{ kI },
	kD_{ kD },
	outputLimitMin_{ outputLimitMin },
	outputLimitMax_{ outputLimitMax },
	tau_{ tau }
{
	// these will change based on driver factor in time.
	std::cout << std::setprecision(3) << "PID values: [ " << kP << ", " << kI << ", " << kD << " ]" << std::endl;

	initialise();
}

void Controller::initialise()
{
	setpoint_ = 0.0;
	proportional_ = 0.0;
	integrator_ = 0.0;
	differentiator_ = 0.0;
	previousError_ = 0.0;
	error_ = 0.0;
	previousMeasurement_ = 0.0;
	measurement_ = 0.0;
	output_ = 0.0;
}

// Anti-wind-up via dynamic integrator clamping
void Controller::clampIntegrator()
{
	double integratorLimitMin, integratorLimitMax;

	if (outputLimitMax_ > proportional_)
	{
		integratorLimitMax = outputLimitMax_ - proportional_;
	}
	else
	{
		integratorLimitMax = 0.0;
	}

	if (outputLimitMin_ < proportional_)
	{
		integratorLimitMin = outputLimitMin_ - proportional_;
	}
	else
	{
		integratorLimitMin = 0.0;
	}

	integrator_ = std::clamp(integrator_, integratorLimitMin, integratorLimitMax);
}

PidData Controller::characteriseController(Controller controller, double setpoint)
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

double Controller::getError() const
{
	return error_;
}

double Controller::update(double deltaSeconds)
{
	double halfT = deltaSeconds * 0.5;

	// Proportional
	proportional_ = kP_ * error_;

	// Integral
	integrator_ = integrator_ + kI_ * halfT * (error_ + previousError_) ;
	clampIntegrator();

	// Differential - Band limited differentiator
	differentiator_ = (kD_ * (measurement_ - previousMeasurement_) + (tau_ - halfT) * differentiator_) / (tau_ + halfT);

	double pid = proportional_ + integrator_ + differentiator_;
	output_ = std::clamp(pid, outputLimitMin_, outputLimitMax_);

	previousError_ = error_;
	error_ = setpoint_ - output_;
	previousMeasurement_ = measurement_;

	return output_;
}

double Controller::getSetPoint() const
{
	return setpoint_;
}

void Controller::setSetPoint(double setpoint)
{
	setpoint_ = setpoint;
	error_ = setpoint_ - output_;
}

double Controller::getMeasurement() const
{
	return setpoint_;
}

void Controller::setMeasurement(double measurement)
{
	measurement_ = measurement;
}


void Controller::setP(double p)
{
	kP_ = p;
}

double Controller::getP() const
{
	return kP_;
}

void Controller::setI(double i)
{
	kI_ = i;
}

double Controller::getI() const
{
	return kI_;
}

void Controller::setD(double d)
{
	kD_ = d;
}

double Controller::getD() const
{
	return kD_;
}