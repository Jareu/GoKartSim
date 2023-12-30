#include "Controller.h"

Controller::Controller(double p, double i, double d) :
	kp_{ p },
	ki_{ i },
	kd_{ d },
    pre_error_{ 0.0 },
    error_{ 0.0 },
    integral_{ 0.0 },
    setpoint_{ 0.0 },
    value_{ 0.0 }
{}

double Controller::update(double dt)
{
    // Calculate error
    error_ = setpoint_ - value_;

    // Proportional term
    double Pout = kp_ * error_;

    // Integral term
    integral_ += error_ * dt;
    double Iout = ki_ * integral_;

    // Derivative term
    double derivative = (error_ - pre_error_) / dt;
    // TODO: introduce moving average filter for derivative
    double Dout = kd_ * derivative;

    // Calculate total output
    value_ = Pout + Iout + Dout;

    // Save error to previous error
    pre_error_ = error_;

    return value_;
}

double Controller::getError() const
{
    return error_;
}

void Controller::reset()
{
    pre_error_ = 0.0;
    integral_ = 0.0;
    setpoint_ = 0.0;
    value_ = 0.0;
}

PidData Controller::characteriseController(Controller controller, double setpoint)
{
    PidData data;

    controller.reset();
    controller.setSetPoint(setpoint);

    double time = 0.0;
    double last_error = 0.0;
    double dError = 0.0;
    do {
        double value = controller.update(CHARACTERISATION_TIMESTEP_);
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

double Controller::getSetPoint() const
{
    return setpoint_;
}

void Controller::setSetPoint(double setpoint)
{
    setpoint_ = setpoint;

    // Calculate error
    error_ = setpoint_ - value_;
}

void Controller::setP(double p)
{
    kp_ = p;
}

double Controller::getP() const
{
   return kp_;
}

void Controller::setI(double i)
{
    ki_ = i;
}

double Controller::getI() const
{
    return ki_;
}

void Controller::setD(double d)
{
    kd_ = d;
}

double Controller::getD() const
{
    return kd_;
}