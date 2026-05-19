package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;

public class IntakeSubsystem extends SubsystemBase {
    IMotorVelocityControl spin;
    IMotorPositionControl pivot;

    public IntakeSubsystem() {
        spin = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            10,
            MotorType.kBrushless,
            false,
            1,
            1,
            450,
            1000,
            0.1,
            0,
            0,
            0,
            0);
        pivot = SparkMaxPositionControl.CreateLinearSparkMaxPositionController(
            13,
            MotorType.kBrushless,
            true,
            16,
            16,
            450,
            1000,
            0.1,
            0,
            0,
            0,
            0);
    }

    public void runSpinVelocity(double value) {
        spin.runVelocity(value);
    }

    public void runPivotPosition(double value) {
        pivot.runPosition(value);
    }
}
