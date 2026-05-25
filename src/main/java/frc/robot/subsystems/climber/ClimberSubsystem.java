package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;

public class ClimberSubsystem extends SubsystemBase {
    IMotorPositionControl motor;

    public ClimberSubsystem() {
        motor = SparkMaxPositionControl.CreateLinearSparkMaxPositionController(
                15,
                MotorType.kBrushless,
                false,
                1,
                1,
                4000,
                1000,
                0.1,
                4,
                0,
                0,
                0);
    }

    public void runPosition(double value) {
        if (value < 0) {
            value = 0;
        }
        if (value > 400) {
            value = 400;
        }

        motor.runPosition(value);
    }

    public void incrementPosition(double changeInPosition) {
        double value = motor.getPosition() + changeInPosition;
        if (value < 0) {
            value = 0;
        }
        if (value > 400) {
            value = 400;
        }

        motor.runPosition(value);
    }
}
