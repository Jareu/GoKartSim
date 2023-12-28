#pragma once
class Controller
{
public:
	Controller() = delete;
	Controller(double p, double i, double d);
	~Controller() = default;

	double update(double dt);

	double getSetPoint();
	void setSetPoint(double setpoint);

	double getP();
	void setP(double p);

	double getI();
	void setI(double i);

	double getD();
	void setD(double d);
private:
	double kp_;
	double ki_;
	double kd_;
	double pre_error_;
	double integral_;
	double setpoint_;
	double value_;
};

