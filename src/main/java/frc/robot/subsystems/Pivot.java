// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  CANSparkMax PivotRightMotor;
  CANSparkMax PivotLeftMotor;
  PIDController controller;
  RelativeEncoder RightThroughbore;
  RelativeEncoder LeftThroughbore;
  /** Creates a new Pivot. */
  
  public Pivot() {
    PivotRightMotor = new CANSparkMax(Constants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    PivotLeftMotor = new CANSparkMax(Constants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    controller = new PIDController(1.0, 0.0, 0.05);
    RightThroughbore = PivotRightMotor.getEncoder();
    LeftThroughbore = PivotLeftMotor.getEncoder();
    PivotLeftMotor.setIdleMode(IdleMode.kBrake);
    PivotRightMotor.setIdleMode(IdleMode.kBrake);
    RightThroughbore.setVelocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
    RightThroughbore.setPositionConversionFactor(Constants.kDriveConversionFactor);

    LeftThroughbore.setVelocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
    LeftThroughbore.setPositionConversionFactor(Constants.kDriveConversionFactor);
  }

  public void pivot(double speed){
    PivotRightMotor.set(speed);
    PivotLeftMotor.set(-speed);
  }
  public void off(){
    PivotRightMotor.set(0);
    PivotLeftMotor.set(0);
  }
  
  public void checkLag(double leftSetpoint, double rightSetpoint) {
    if (Math.abs(RightThroughbore.getPosition() - LeftThroughbore.getPosition()) > 3) {
      if (RightThroughbore.getPosition() < LeftThroughbore.getPosition()) {
        PivotRightMotor.set(controller.calculate(RightThroughbore.getPosition(), rightSetpoint));
      }
      else {
        PivotLeftMotor.set(controller.calculate(LeftThroughbore.getPosition(), leftSetpoint));
      }
    }
    else {
      PivotRightMotor.set(controller.calculate(RightThroughbore.getPosition(), rightSetpoint));
      PivotLeftMotor.set(controller.calculate(LeftThroughbore.getPosition(), leftSetpoint));
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
