// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.services.IndexerService;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIndexer extends InstantCommand {
  IndexerSubsystem m_indexer;

  public RunIndexer(IndexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IndexerService.runIndexer(m_indexer);
  }
}
