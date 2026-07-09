package frc.robot.services;

import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeService {
    public static void runIntakeSpin(IntakeSubsystem intake) {
        intake.runSpinDutyCycle(1);
    }

    public static void stopIntakeSpin(IntakeSubsystem intake) {
        intake.runSpinVelocity(0);
    }

    public static void IntakePivotGround(IntakeSubsystem intake) {
        intake.runPivotPosition(IntakeServiceConstants.INTAKE_PIVOT_GROUND);
    }

    public static void IntakePivotUp(IntakeSubsystem intake) {
        intake.runPivotPosition(IntakeServiceConstants.INTAKE_PIVOT_UP);
    }

    public static void IntakePivotMiddle(IntakeSubsystem intake) {
        intake.runPivotPosition(IntakeServiceConstants.INTAKE_PIVOT_MIDDLE);
    }

    public static boolean isIntakeMiddle(IntakeSubsystem intake) {
        if (intake.getPivotPosition() < (IntakeServiceConstants.INTAKE_PIVOT_MIDDLE + IntakeServiceConstants.DEADZONE / 2) ||
            intake.getPivotPosition() < (IntakeServiceConstants.INTAKE_PIVOT_MIDDLE - IntakeServiceConstants.DEADZONE / 2)) {
            
            return true;
        }
        return false;
    }

    public static boolean isIntakeGround(IntakeSubsystem intake) {
        if (intake.getPivotPosition() < (IntakeServiceConstants.INTAKE_PIVOT_GROUND + IntakeServiceConstants.DEADZONE)) {
            return true;
        }

        return false;
    }
}
