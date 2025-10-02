#include "pid_controller/pid.hpp"

//Constructor - initilising from provided gains
PID::PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

// Define the compute method to produce output of pid controller
double PID::compute(double target, double current, double dt) {

    // begin by calcualting error
    double error = target - current;

    // from (current) error, find the proportional term
    double P = kp_ * error;

    // from (current) error, find the integral term
    integral_ += error * dt;
    double I = ki_ * integral_;

    // from (current) error, find the derivative term
    double derivative = (error - prev_error_) / dt;
    double D = kd_ * derivative;

    // set the previous error to current error for next derivative calculation
    prev_error_ = error;

    // output is the sum of terms
    double output = P + I + D;
    return output;
}