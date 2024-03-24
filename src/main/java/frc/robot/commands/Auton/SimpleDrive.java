// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;


public class SimpleDrive extends Command {
  /** Creates a new SimpleDrive. */
  DrivetrainSubsystem m_drive;
  Timer timer;
  public SimpleDrive(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(new ChassisSpeeds(0.5, 0, 0), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 2.7;
  }
}
