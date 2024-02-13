// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

import java.lang.Math;

public class RetractPivot extends Command {

  CANSparkMax pivotRight;
  CANSparkMax pivotLeft;
  PIDController controller;
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;
  Pivot m_pivot;
  
  /** Creates a new RetractPivot. */
  public RetractPivot(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.checkLag(0,0);
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