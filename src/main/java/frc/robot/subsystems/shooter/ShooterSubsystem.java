package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;
import frc.robot.motors.tuning.PositionGainsTuner;
import frc.robot.motors.tuning.VelocityGainsTuner;

public class ShooterSubsystem extends SubsystemBase {
    IMotorVelocityControl shooter;
    IMotorVelocityControl kicker;
    IMotorPositionControl turret;

    VelocityGainsTuner kicker_tuner;
    PositionGainsTuner turret_tuner;

    public ShooterSubsystem() {
        shooter = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            14,
            MotorType.kBrushless,
            false,
            9,
            1,
            6000,
            2500,
            0.1,
            0.0004,
            0,
            0.00203,
            0);

        kicker  = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            12,
            MotorType.kBrushless,
            true,
            9,
            1,
            1000,
            3000,
            0.1,
            0,
            0.15,
            0.00208,
            0);

        kicker_tuner = new VelocityGainsTuner("Shooter/Kicker", kicker, 0, 0.15, 0.00208, 0);

        turret  = SparkMaxPositionControl.CreateLinearSparkMaxPositionController(
            9,
            MotorType.kBrushless,
            true,
            9,
            1,
            1000,
            3000,
            0.1,
            0.1,
            0,
            0,
            0);
            
        turret_tuner = new PositionGainsTuner("Shooter/Turret", turret, 0.1, 0, 0, 0, 0, 0);  
    }

    public void runShooterVelocity(double value) {
        shooter.runVelocity(value);
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }

    public void runKickerVelocity(double value) {
        kicker.runVelocity(value);
    }

    public double getKickerVelocity() {
        return kicker.getVelocity();
    }

    public void runTurretPosition(double value) {
        turret.runPosition(value);
    }

    public double getTurretPosition() {
        return turret.getPosition();
    }

    @Override
    public void periodic() {
        kicker_tuner.update();
        turret_tuner.update();
    }
}
