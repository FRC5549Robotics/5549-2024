// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;


public class PIDShooterAuton extends Command {
  /** Creates a new PIDShooter. */
  Shooter m_shooter;
  Double shootSpeed;
  Limelight m_limelight;
  Timer timer;
  public PIDShooterAuton(Shooter shooter, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_limelight = limelight;
    timer = new Timer();
    timer.reset();
    timer.start();
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootSpeed = m_limelight.getDesiredRPM();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_shooter.on(shootSpeed);
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
    return timer.get()>3;
  }
}