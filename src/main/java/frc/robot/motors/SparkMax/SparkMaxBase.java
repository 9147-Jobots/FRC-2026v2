package frc.robot.motors.SparkMax;

import frc.robot.motors.IMotorBase;

import com.revrobotics.spark.SparkMax;

public abstract class SparkMaxBase implements IMotorBase {

    // Expose the underlying SparkMax to subclasses for configuration and use.
    // Kept protected so subclasses (e.g., position/velocity controllers) can
    // reuse the same hardware instance instead of creating duplicates.
    protected final SparkMax sparkMax;

    public SparkMaxBase(SparkMax sparkMax) {
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
    public void runVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }
}