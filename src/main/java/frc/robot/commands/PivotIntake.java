// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotTarget;

public class PivotIntake extends Command {
  /** Creates a new PivotIntake. */
  Pivot m_pivot;
  Pivot.PivotTarget target;
  double leftSetpoint, rightSetpoint;
  public PivotIntake(Pivot pivot, Pivot.PivotTarget Target) {
    m_pivot = pivot;
    target = Target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(target == PivotTarget.Intake){
      leftSetpoint = Constants.PIVOT_LEFT_INTAKE_SETPOINT;
      rightSetpoint = Constants.PIVOT_RIGHT_INTAKE_SETPOINT;
    }
    else if(target == PivotTarget.Retracted){
      leftSetpoint = Constants.PIVOT_LEFT_RETRACTED_SETPOINT;
      rightSetpoint = Constants.PIVOT_RIGHT_RETRACTED_SETPOINT;
    }
    else if(target == PivotTarget.Amp) {
      leftSetpoint = Constants.PIVOT_LEFT_AMP_SETPOINT;
      rightSetpoint = Constants.PIVOT_RIGHT_AMP_SETPOINT;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.checkLag(leftSetpoint, rightSetpoint);
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
