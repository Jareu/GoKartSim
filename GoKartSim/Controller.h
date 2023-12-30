#pragma once
#include <vector>
#include "globals.h"

class Controller
{
public:
	Controller() = delete;
	Controller(double p, double i, double d);
	~Controller() = default;

	double update(double dt);
	void reset();

	double getError() const;

	double getSetPoint() const;
	void setSetPoint(double setpoint);

	double getP() const;
	void setP(double p);

	double getI() const;
	void setI(double i);

	double getD() const;
	void setD(double d);
	static PidData characteriseController(Controller controller, double setpoint);
private:
	double kp_;
	double ki_;
	double kd_;
	double pre_error_;
	double error_;
	double integral_;
	double setpoint_;
	double value_;
	static constexpr double CHARACTERISATION_TIMESTEP_ = 0.01;
	static constexpr double CHARACTERISATION_THRESHOLD_ = 0.00001;
};

