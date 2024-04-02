// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.CommandVariants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;

public class IntakeAuton extends Command {
  /** Creates a new AutoIndex. */
  Timer m_timer;
  double startTime;
  Intake m_intake;
  public IntakeAuton(Intake intake, Timer timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    // analog = new AnalogInput(0);
    m_intake = intake;
    m_timer = timer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = m_timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() - startTime > 1.5;
  }
}
