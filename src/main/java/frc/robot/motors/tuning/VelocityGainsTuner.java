package frc.robot.motors.tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.IMotorVelocityControl;

public class VelocityGainsTuner {

    private final String prefix;
    private final IMotorVelocityControl motor;

    private double kP, kS, kV, kA;
    private double testSetpoint;

    public VelocityGainsTuner(String prefix, IMotorVelocityControl motor,
            double kP, double kS, double kV, double kA) {
        this.prefix = prefix;
        this.motor = motor;
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.testSetpoint = 0;

        SmartDashboard.putBoolean(prefix + "/isTuning", false);
        SmartDashboard.putNumber(prefix + "/testSetpoint", testSetpoint);
        SmartDashboard.putNumber(prefix + "/kP", kP);
        SmartDashboard.putNumber(prefix + "/kS", kS);
        SmartDashboard.putNumber(prefix + "/kV", kV);
        SmartDashboard.putNumber(prefix + "/kA", kA);
    }

    public void update() {
        double newKP = SmartDashboard.getNumber(prefix + "/kP", kP);
        double newKS = SmartDashboard.getNumber(prefix + "/kS", kS);
        double newKV = SmartDashboard.getNumber(prefix + "/kV", kV);
        double newKA = SmartDashboard.getNumber(prefix + "/kA", kA);

        if (newKP != kP || newKS != kS || newKV != kV || newKA != kA) {
            kP = newKP;
            kS = newKS;
            kV = newKV;
            kA = newKA;
            motor.updateGains(kP, kS, kV, kA);
        }

        boolean isTuning = SmartDashboard.getBoolean(prefix + "/isTuning", false);
        if (isTuning) {
            testSetpoint = SmartDashboard.getNumber(prefix + "/testSetpoint", testSetpoint);
            motor.runVelocity(testSetpoint);
            SmartDashboard.putNumber(prefix + "/velocity", motor.getVelocity());
            SmartDashboard.putNumber(prefix + "/currentSetpoint", motor.getSetpoint());
        }
    }
}
