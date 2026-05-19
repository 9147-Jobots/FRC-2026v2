package frc.robot.services;

import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeService {
    public static void runIntakeSpin(IntakeSubsystem intake) {
        intake.runSpinVelocity(5); // TO BE TUNED
    }

    public static void stopIntakeSpin(IntakeSubsystem intake) {
        intake.runSpinVelocity(0);
    }

    public static void IntakePivotGround(IntakeSubsystem intake) {
        intake.runPivotPosition(5); // TO BE TUNED
    }

    public static void IntakePivotUp(IntakeSubsystem intake) {
        intake.runPivotPosition(5); // TO BE TUNED
    }

    public static void IntakePivotRest(IntakeSubsystem intake) {
        intake.runPivotPosition(5); // TO BE TUNED
    }
}
