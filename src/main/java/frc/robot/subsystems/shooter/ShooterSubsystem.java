package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;

public class ShooterSubsystem extends SubsystemBase {
    IMotorVelocityControl shooter;
    IMotorVelocityControl kicker;
    IMotorPositionControl turret;

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
            1000,
            0.1,
            5,
            0,
            0,
            0);

        turret  = SparkMaxPositionControl.CreateLinearSparkMaxPositionController(
            9,
            MotorType.kBrushless,
            true,
            9,
            1,
            1000,
            60000,
            0.1,
            0.1,
            0,
            0,
            0);
    }

    public void runShooterVelocity(double value) {
        shooter.runVelocity(value);
    }

    public void runKickerVelocity(double value) {
        kicker.runVelocity(value);
    }
    public void runTurretPosition(double value) {
        turret.runPosition(value);
    }

}
