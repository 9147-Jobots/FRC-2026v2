package frc.robot.motors.TalonFX;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.motors.IMotorBase;

public abstract class TalonFXBase implements IMotorBase {
    
    protected final TalonFX talonFX;

    protected TalonFXBase(TalonFX talonFX) {
        this.talonFX = talonFX;
    }

    @Override
    public double getCurrent() {
        return talonFX.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return talonFX.getDeviceTemp().getValueAsDouble();
    }
 
    @Override
    public double getVoltage() {
        return talonFX.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void runVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    } 
}
