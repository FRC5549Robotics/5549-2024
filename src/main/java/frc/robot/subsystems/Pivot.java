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

  CANSparkMax pivotRight;
  CANSparkMax pivotLeft;
  PIDController controller;
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;
  /** Creates a new Pivot. */
  
  public Pivot() {
    pivotRight = new CANSparkMax(0, MotorType.kBrushless);
    pivotLeft = new CANSparkMax(1, MotorType.kBrushless);
    controller = new PIDController(1.0, 0.0, 0.05);
    rightEncoder = pivotRight.getEncoder();
    leftEncoder = pivotLeft.getEncoder();
    rightEncoder.setVelocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
    rightEncoder.setPositionConversionFactor(Constants.kDriveConversionFactor);

    leftEncoder.setVelocityConversionFactor(Constants.kDriveConversionFactor / 60.0);
    leftEncoder.setPositionConversionFactor(Constants.kDriveConversionFactor);
  }

  public void pivot(double speed){
    pivotRight.set(speed);
    pivotLeft.set(-speed);
  }
  public void off(){
    pivotRight.set(0);
    pivotLeft.set(0);
  }
  
  public void checkLag() {
    if (Math.abs(rightEncoder.getPosition() - leftEncoder.getPosition()) > 3) {
      if (rightEncoder.getPosition() < leftEncoder.getPosition()) {
        pivotRight.set(controller.calculate(rightEncoder.getPosition(), Constants.PIVOT_DEPLOY_SETPOINT));
      }
      else {
        pivotLeft.set(controller.calculate(leftEncoder.getPosition(), Constants.PIVOT_DEPLOY_SETPOINT));
      }
    }
    else {
      pivotRight.set(controller.calculate(rightEncoder.getPosition(), Constants.PIVOT_DEPLOY_SETPOINT));
      pivotLeft.set(controller.calculate(leftEncoder.getPosition(), Constants.PIVOT_DEPLOY_SETPOINT));
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
