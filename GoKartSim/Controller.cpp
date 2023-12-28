#include "Controller.h"

Controller::Controller(double p, double i, double d) :
	kp_{ p },
	ki_{ i },
	kd_{ d },
    pre_error_{ 0.0 },
    integral_{ 0.0 },
    setpoint_{ 0.0 },
    value_{ 0.0 }
{}

double Controller::update(double dt)
{
    // Calculate error
    double error = setpoint_ - value_;

    // Proportional term
    double Pout = kp_ * error;

    // Integral term
    integral_ += error * dt;
    double Iout = ki_ * integral_;

    // Derivative term
    double derivative = (error - pre_error_) / dt;
    // TODO: introduce moving average filter for derivative
    double Dout = kd_ * derivative;

    // Calculate total output
    value_ = Pout + Iout + Dout;

    // Save error to previous error
    pre_error_ = error;

    return value_;
}

double Controller::getSetPoint()
{
    return setpoint_;
}

void Controller::setSetPoint(double setpoint)
{
    setpoint_ = setpoint;
}

void Controller::setP(double p)
{
    kp_ = p;
}

double Controller::getP()
{
   return kp_;
}

void Controller::setI(double i)
{
    ki_ = i;
}

double Controller::getI()
{
    return ki_;
}

void Controller::setD(double d)
{
    kd_ = d;
}

double Controller::getD()
{
    return kd_;
}