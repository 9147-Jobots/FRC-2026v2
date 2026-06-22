package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;
import frc.robot.motors.tuning.PositionGainsTuner;
import frc.robot.motors.tuning.VelocityGainsTuner;

public class IntakeSubsystem extends SubsystemBase {
    IMotorVelocityControl spin;
    IMotorPositionControl pivot;
    VelocityGainsTuner spin_tuner;
    PositionGainsTuner pivot_tuner;


    public IntakeSubsystem() {
        spin = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            10,
            MotorType.kBrushless,
            false,
            1,
            1,
            3000,
            4000,
            0.1,
            0,
            0.18,
            0.002025,
            0);
        pivot = SparkMaxPositionControl.CreatePivotFFSparkMaxPositionController(
            13,
            MotorType.kBrushless,
            false,
            16*9/11,
            1,
            7000,
            3000,
            1,
            0.14,
            0,
            0,
            0,
            0.42);

        pivot.zeroPosition(132);

        spin_tuner = new VelocityGainsTuner("Intake/Spin", spin, 0.0, 0.18, 0.002025, 0);
        pivot_tuner = new PositionGainsTuner("Intake/Pivot", pivot, 0.14, 0, 0.0, 0, 0, 0.42);
    }

    public void runSpinVelocity(double value) {
        spin.runVelocity(value);
    }

    public void runPivotPosition(double value) {
        pivot.runPosition(value);
    }

    public void runSpinDutyCycle(double value) {
        spin.runDutyCycle(value);
    }

    public double getPivotPosition() {
        return pivot.getPosition();
    }

    @Override
    public void periodic() {
        pivot_tuner.update();
        spin_tuner.update();
    }
}
