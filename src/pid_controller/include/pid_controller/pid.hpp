#ifndef PID_HPP_
#define PID_HPP_

class PID {
    public: 
        // constructor taking in the PID coefficients
        PID(double kp, double ki, double kd);

        // method to compute the control output
        double compute(double setpoint, double measurement, double dt);


    private: 
        double kp_, ki_, kd_; // gains
        double integral_; // integral term
        double prev_error_; // previous error for derivative term
    }

#endif // PID_HPP_