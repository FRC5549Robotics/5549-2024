// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberAnalog extends Command {
  public enum Side{
    Right,
    Left
  }
  /** Creates a new PivotIntake. */
  Climber m_climber;
  CommandXboxController m_controller;
  Side m_side;
  public ClimberAnalog(Climber climber, CommandXboxController controller, Side side) {
    m_climber = climber;
    m_controller = controller;
    m_side = side;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_side == Side.Left){
      m_climber.runLeftClimber(m_controller.getLeftY()*Constants.CLIMBER_SCALING_FACTOR);
    }
    else{
      m_climber.runRightClimber(m_controller.getRightY()*Constants.CLIMBER_SCALING_FACTOR);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}