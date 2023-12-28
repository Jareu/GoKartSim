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

double Controller::getError() const
{
    return pre_error_;
}

void Controller::reset()
{
    pre_error_ = 0.0;
    integral_ = 0.0;
    setpoint_ = 0.0;
    value_ = 0.0;
}

std::vector<Controller::CharacterisationData> Controller::characteriseController(Controller controller, double setpoint)
{
    std::vector<Controller::CharacterisationData> data;
    controller.reset();
    controller.setSetPoint(setpoint);

    for (double t = 0.0; controller.getError() < CHARACTERISATION_THRESHOLD_; t += CHARACTERISATION_TIMESTEP_) {
        double value = controller.update(CHARACTERISATION_TIMESTEP_);
        data.emplace_back(Controller::CharacterisationData{ t, value });
    }

    return data;
}

double Controller::getSetPoint() const
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