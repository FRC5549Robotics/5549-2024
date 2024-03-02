// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Deflectorinator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Deflectorinator.DeflectorinatorTarget;
import frc.robot.subsystems.Pivot.PivotTarget;

public class DeflectRotate extends Command {
  /** Creates a new PivotIntake. */
  Deflectorinator m_deflectorinator;
  Deflectorinator.DeflectorinatorTarget target;
  double setpoint;
  public DeflectRotate(Deflectorinator deflectorinator, Deflectorinator.DeflectorinatorTarget Target) {
    m_deflectorinator = deflectorinator;
    target = Target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(target == DeflectorinatorTarget.AmpShooting){
      setpoint = Constants.DEFLECTORINATOR_OUT_SETPOINT;
    }
    else if(target == DeflectorinatorTarget.Retracted){
      setpoint = Constants.DEFLECTORINATOR_RETRACTED_SETPOINT;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_deflectorinator.deflectorinatorEncoderPivot(setpoint);
    System.out.println("running deflect");
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
