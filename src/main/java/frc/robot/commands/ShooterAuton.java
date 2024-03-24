// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class ShooterAuton extends Command {
  Timer timer;
  Shooter m_shooter;
  /** Creates a new ShooterAuton. */
  public ShooterAuton(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    timer.reset();
    timer.start();

    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.on(Constants.SHOOTER_SET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 3);
  }
}
