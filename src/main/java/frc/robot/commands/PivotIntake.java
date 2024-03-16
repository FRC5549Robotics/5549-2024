// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotTarget;

public class PivotIntake extends Command {
  /** Creates a new PivotIntake. */
  Pivot m_pivot;
  Indexer m_indexer;
  Intake m_intake;
  Shooter m_shooter;
  Pivot.PivotTarget target;
  double leftSetpoint, rightSetpoint;
  public PivotIntake(Pivot pivot, Pivot.PivotTarget Target, Indexer indexer, Intake intake, Shooter shooter) {
    m_pivot = pivot;
    target = Target;
    m_indexer = indexer;
    m_intake = intake;
    m_shooter = shooter;
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
    if(Math.abs(m_pivot.getRightPosition()-Constants.PIVOT_RIGHT_INTAKE_SETPOINT) < 3 && 
      Math.abs(m_pivot.getLeftPosition()-Constants.PIVOT_LEFT_INTAKE_SETPOINT) < 3){
        m_indexer.indexIn();
        m_intake.intake(1);
    }
    else{
      m_indexer.off();
      m_intake.off();
      m_pivot.autonPivotIn();
      m_shooter.shooterAmp();
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
