package frc.robot.motors;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

public interface IMotorDiagnostics {

    /**
     * Returns the current velocity of the motor.
     * @return the current velocity of the motor
     */
    public double getVelocity();
        
    /**
     * Returns the current position of the motor.
     * @return the current position of the motor
     */
    public double getPosition();

    /**
     * Returns the current drawn by the motor in amps.
     * @return the current drawn by the motor in amps
     */
    public double getCurrent();

    /**
     * Returns the temperature of the motor in degrees Celsius.
     * @return the temperature of the motor in degrees Celsius
     */
    public double getTemperature();

    /**
     * Returns the voltage supplied to the motor in volts.
     * @return the voltage supplied to the motor in volts
     */
    public double getVoltage();

    /**
     * Returns the current faults of the motor.
     * @return the current faults of the motor
     */
    public Faults getFaults();

    /**
     * Returns the current warnings of the motor.
     * @return the current warnings of the motor
     */
    public Warnings getWarnings();
}
