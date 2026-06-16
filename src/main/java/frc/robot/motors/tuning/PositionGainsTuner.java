package frc.robot.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorPositionControl.PositionControlMode;

public class PositionGainsTuner {

    private final String prefix;
    private final IMotorPositionControl motor;

    private double kP, kS, kV, kA, kG, kCos;
    private double testSetpoint;

    public PositionGainsTuner(String prefix, IMotorPositionControl motor,
            double kP, double kS, double kV, double kA, double kG, double kCos) {
        this.prefix = prefix;
        this.motor = motor;
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        this.kCos = kCos;
        this.testSetpoint = 0;

        SmartDashboard.putBoolean(prefix + "/isTuning", false);
        SmartDashboard.putNumber(prefix + "/testSetpoint", testSetpoint);
        SmartDashboard.putNumber(prefix + "/kP", kP);
        SmartDashboard.putNumber(prefix + "/kS", kS);
        SmartDashboard.putNumber(prefix + "/kV", kV);
        SmartDashboard.putNumber(prefix + "/kA", kA);
        SmartDashboard.putNumber(prefix + "/kG", kG);
        SmartDashboard.putNumber(prefix + "/kCos", kCos);
    }

    public void update() {
        double newKP   = SmartDashboard.getNumber(prefix + "/kP",   kP);
        double newKS   = SmartDashboard.getNumber(prefix + "/kS",   kS);
        double newKV   = SmartDashboard.getNumber(prefix + "/kV",   kV);
        double newKA   = SmartDashboard.getNumber(prefix + "/kA",   kA);
        double newKG   = SmartDashboard.getNumber(prefix + "/kG",   kG);
        double newKCos = SmartDashboard.getNumber(prefix + "/kCos", kCos);

        if (newKP != kP || newKS != kS || newKV != kV || newKA != kA
                || newKG != kG || newKCos != kCos) {
            kP   = newKP;
            kS   = newKS;
            kV   = newKV;
            kA   = newKA;
            kG   = newKG;
            kCos = newKCos;
            motor.updateGains(kP, kS, kV, kA, kG, kCos);
        }

        boolean isTuning = SmartDashboard.getBoolean(prefix + "/isTuning", false);
        if (isTuning) {
            testSetpoint = SmartDashboard.getNumber(prefix + "/testSetpoint", testSetpoint);
            motor.runPosition(testSetpoint, PositionControlMode.kDirectPID);
            SmartDashboard.putNumber(prefix + "/position", motor.getPosition());
            SmartDashboard.putNumber(prefix + "/currentSetpoint", motor.getSetpoint());
        }
    }
}
