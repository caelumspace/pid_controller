#include <iostream>
#include <chrono>
#include <cmath>

// -------------------------------------------------------------
// PID CONTROLLER (same as provided)
// -------------------------------------------------------------
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

// -------------------------------------------------------------
// SIMULATED STEERING WHEEL CLASS
// -------------------------------------------------------------
class SteeringWheel
{
public:
    SteeringWheel() : current_angle_(0.0) {}

    /**
     * Set the steering angle (in degrees or radians; choose a convention).
     * For real hardware, this would interface with the car's steering actuator.
     */
    void setAngle(double angle)
    {
        // In a real system, you would send 'angle' to your steering actuator.
        // Here, we just store it.
        current_angle_ = angle;
    }

    /**
     * Get the current steering angle.
     */
    double getAngle() const
    {
        return current_angle_;
    }

private:
    double current_angle_;  // The current steering angle
};

// -------------------------------------------------------------
// MAIN FUNCTION (SIMULATION DEMO)
// -------------------------------------------------------------
int main()
{
    // 1. Create a PID controller with some initial gains and time step
    double kp = 0.4;   // Proportional gain
    double ki = 0.01;  // Integral gain
    double kd = 0.1;   // Derivative gain
    double dt = 0.05;  // 50 ms sample time

    PIDController pidController(kp, ki, kd, dt);

    // 2. Create a SteeringWheel object to represent our car's steering actuator
    SteeringWheel steering;

    // 3. Our goal: maintain a "lateral offset" of 0 (stay in the lane center).
    double setpoint = 0.0;  

    // 4. Initialize a simulated lateral position (error source). 
    //    In a real car, you'd read this from sensors (camera, LiDAR, etc.).
    double measured_position = 1.0;  // Start 1 meter offset to one side
    double error = 0.0;

    // 5. Run a control loop to simulate the car moving as we apply steering corrections
    for (int step = 0; step < 50; ++step)
    {
        // Calculate the current error (difference from setpoint)
        error = setpoint - measured_position;

        // Use the PID controller to compute a new steering command
        double steering_command = pidController.computeControl(error);

        // Command the steering actuator (simulated) to set this steering angle
        // (For simplicity, we treat the control output as the direct angle.)
        steering.setAngle(steering_command);

        // In a real system, the new steering angle would gradually affect the car's heading.
        // For our simplified simulation, assume each step of steering_command
        // reduces the lateral offset proportionally:
        measured_position += steering_command * 0.1;

        // Print or log the results
        std::cout << "Step: " << step
                  << " | Error: " << error
                  << " | Steering Command: " << steering_command
                  << " | Applied Steering Angle: " << steering.getAngle()
                  << " | Measured Position: " << measured_position
                  << std::endl;
    }

    return 0;
}
