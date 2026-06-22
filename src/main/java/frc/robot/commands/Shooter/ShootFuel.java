// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.services.ShooterService;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  CommandSwerveDrivetrain m_drive;
  IndexerSubsystem m_indexer;
  ShooterSubsystem m_shooter;

  /** Creates a new ShootFuel. */
  public ShootFuel(CommandSwerveDrivetrain drive, IndexerSubsystem indexer, ShooterSubsystem shooter) {
    m_drive = drive;
    m_indexer = indexer;
    m_shooter = shooter;
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterService.AlignTurretPeriodic(m_shooter, m_drive);
    //ShooterService.shootFuel(m_shooter, m_indexer, m_drive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterService.stopShooterKicker(m_shooter);
    ShooterService.stopShooterShooter(m_shooter);
    ShooterService.stopIndexer(m_indexer);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
