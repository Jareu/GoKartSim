#pragma once

#include "globals.h"

class Controller {
public:

	Controller();
	Controller(double kP, double kI, double kD, double outputLimitMin, double outputLimitMax, double tau);

	void initialise();
	double update(double deltaSeconds);
	double getError() const;

	double getSetPoint() const;
	void setSetPoint(double setpoint);

	double getMeasurement() const;
	void setMeasurement(double measurement);

	double getP() const;
	void setP(double p);

	double getI() const;
	void setI(double i);

	double getD() const;
	void setD(double d);

	static PidData characteriseController(Controller controller, double setpoint);
private:
	void clampIntegrator();

	double kP_;
	double kI_;
	double kD_;

	// Derivative low-pass filter time constant
	double tau_;

	double outputLimitMin_;
	double outputLimitMax_;

	// Controller "memory"
	double setpoint_;
	double proportional_;
	double integrator_;
	double differentiator_;
	double previousError_;
	double error_;
	double previousMeasurement_;
	double measurement_;
	double output_;

	static constexpr double CHARACTERISATION_TIMESTEP_ = 0.01;
	static constexpr double CHARACTERISATION_THRESHOLD_ = 0.00001;

	static constexpr double DEFAULT_KP = 0.3;
	static constexpr double DEFAULT_KI = 0.5;
	static constexpr double DEFAULT_KD = 0.1;
	static constexpr double DEFAULT_MIN = -1000.0;
	static constexpr double DEFAULT_MAX = 1000.0;
	static constexpr double DEFAULT_TAU = 0.25;
};
