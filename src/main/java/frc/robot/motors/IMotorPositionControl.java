package frc.robot.motors;

public interface IMotorPositionControl extends IMotorBase {
    
    /**
     * Runs the motor to the specified position setpoint.
     * @param setpoint the position setpoint to run the motor to
     */
    public void runPosition(double setpoint);

    /**
     * Returns the current setpoint of the motor. This is not necessarily the same as the current position of the motor, as the motor may not be at the setpoint yet.
     * @return the current setpoint of the motor
     */
    public double getSetpoint();
    
    /**
     * Returns whether the motor is at its target position.
     * @return true if the motor is at its target position, false otherwise
     */
    public boolean isAtTarget();

    /**
     * Returns whether the motor is within the specified tolerance of its target position.
     * @param tolerance the tolerance to check against
     * @return true if the motor is within the specified tolerance of its target position, false otherwise
     */
    public boolean isAtTarget(double tolerance);

    /**
     * Zeros the position of the motor.
     * @param position the position to set as zero
     */
    public void zeroPosition(double position);

    /**
     * Stops the motor.
     */
    public void stop();
}
