package frc.robot.motors.SparkMax;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.robot.motors.IMotorDiagnostics;

import com.revrobotics.spark.SparkMax;

public abstract class SparkMaxDiagnostics implements IMotorDiagnostics {

    // Expose the underlying SparkMax to subclasses for configuration and use.
    // Kept protected so subclasses (e.g., position/velocity controllers) can
    // reuse the same hardware instance instead of creating duplicates.
    protected final SparkMax sparkMax;

    @Override
    public abstract double getVelocity();

    @Override
    public abstract double getPosition();

    public SparkMaxDiagnostics(SparkMax sparkMax) {
        this.sparkMax = sparkMax;
    }

    @Override
    public double getCurrent() {
        return sparkMax.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return sparkMax.getMotorTemperature();
    }

    @Override
    public double getVoltage() {
        return sparkMax.getBusVoltage();
    }

    @Override
    public Faults getFaults() {
        return sparkMax.getFaults();
    }

    @Override
    public Warnings getWarnings() {
        return sparkMax.getWarnings();
    }
}