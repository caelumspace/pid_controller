#include <iostream>
#include <chrono>

class PIDController
{
public:
    /**
     * Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param dt Time step (in seconds)
     */
    PIDController(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          integral_(0.0), prev_error_(0.0), first_run_(true)
    {
    }

    /**
     * Computes the PID control output given the current error.
     * @param error The difference between the desired setpoint and the actual measurement
     * @return Control output (e.g., steering angle, throttle, etc.)
     */
    double computeControl(double error)
    {
        // On the first run, there's no previous error to compare to
        if (first_run_)
        {
            prev_error_ = error;
            first_run_ = false;
        }

        // Accumulate the integral term
        integral_ += error * dt_;

        // Calculate the derivative term
        double derivative = (error - prev_error_) / dt_;

        // PID formula
        double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

        // Store current error as previous error for next iteration
        prev_error_ = error;

        return output;
    }

    // Optional: Setter functions to allow runtime tuning of PID gains
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }
    void setDt(double dt) { dt_ = dt; }

private:
    double kp_;        // Proportional gain
    double ki_;        // Integral gain
    double kd_;        // Derivative gain
    double dt_;        // Time step

    double integral_;   // Accumulated integral
    double prev_error_; // Previous error for derivative calculation
    bool first_run_;    // Flag to indicate first compute cycle
};

int main()
{
    // Example usage:
    // Suppose we want to control the steering to maintain a lane center.
    // We'll simulate a simple scenario where the setpoint is 0 (no lateral error)
    // and we observe some measured lateral position from sensors.

    // 1. Create a PID controller with chosen gains
    double kp = 0.4;   // Tune as needed
    double ki = 0.01;  // Tune as needed
    double kd = 0.1;   // Tune as needed
    double dt = 0.05;  // 50 ms sample time, for example

    PIDController pidController(kp, ki, kd, dt);

    // 2. Set your target value (e.g., 0 error for lane-keeping)
    double setpoint = 0.0;  

    // 3. Simulate a measured lateral offset (error) that the PID controller will try to correct
    //    In a real scenario, this would come from sensors/fusion (e.g., camera, LiDAR).
    double measured_position = 1.0;  // Start with some offset from center
    double error = 0.0;
    double control_output = 0.0;

    // 4. Simulate or loop through your control cycle
    //    In a real system, this would typically be in a timed loop based on sensor updates.
    for (int step = 0; step < 50; ++step)
    {
        // Calculate current error
        error = setpoint - measured_position;

        // Compute the control output (e.g., steering angle or rate)
        control_output = pidController.computeControl(error);

        // Apply the control output to your system model or real actuator:
        // Here we do a simple simulation of the effect:
        // Assume that the vehicle's lateral position changes by the control output
        // multiplied by some factor (this is just illustrative).
        measured_position += control_output * 0.1;

        // Print or log the results
        std::cout << "Step: " << step
                  << " | Error: " << error
                  << " | Control Output: " << control_output
                  << " | Measured Position: " << measured_position
                  << std::endl;
    }

    return 0;
}
