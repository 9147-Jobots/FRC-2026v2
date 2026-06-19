package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.services.IntakeService;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeDown extends InstantCommand {

    private IntakeSubsystem m_intake;


    public IntakeDown(IntakeSubsystem intake) {
        addRequirements(intake);

        m_intake = intake;
    }

    @Override
    public void initialize() {
        IntakeService.IntakePivotGround(m_intake);
    }
}
