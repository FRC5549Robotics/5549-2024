// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj.AnalogInput;

public class AutoIndex extends Command {
  /** Creates a new AutoIndex. */
  AnalogInput analog;
  Indexer m_indexer;
  Pivot m_pivot;
  boolean end;
  public AutoIndex(Indexer indexer, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    analog = new AnalogInput(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    end = m_indexer.indexIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
