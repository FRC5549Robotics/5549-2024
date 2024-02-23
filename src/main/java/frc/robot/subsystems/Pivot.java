// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

  
public class Pivot extends SubsystemBase {

  public enum PivotTarget{
    Retracted,
    Intake,
  }

  CANSparkMax PivotRightMotor;
  CANSparkMax PivotLeftMotor;
  PIDController controller;
  DutyCycleEncoder RightThroughbore;
  DutyCycleEncoder LeftThroughbore;
  /** Creates a new Pivot. */
  
  public Pivot() {
    PivotRightMotor = new CANSparkMax(Constants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    PivotLeftMotor = new CANSparkMax(Constants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    controller = new PIDController(0, 0.0, 0.0);
    RightThroughbore = new DutyCycleEncoder(4);
    LeftThroughbore = new DutyCycleEncoder(0);
    RightThroughbore.setPositionOffset(0.9015);
    LeftThroughbore.setPositionOffset(0.3688);
    RightThroughbore.setDistancePerRotation(360);
    LeftThroughbore.setDistancePerRotation(360);
    PivotLeftMotor.setIdleMode(IdleMode.kBrake);
    PivotRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void pivot(double speed){
  
    PivotRightMotor.set(speed);
    PivotLeftMotor.set(-speed);

  }
  public void off(){
    PivotRightMotor.set(0);
    PivotLeftMotor.set(0);
  }
  
  public void checkLag(double setpoint) {
    if (Math.abs(RightThroughbore.getDistance() - LeftThroughbore.getDistance()) > 5) {
      if (RightThroughbore.getDistance() < LeftThroughbore.getDistance()) {
        PivotRightMotor.set(controller.calculate(RightThroughbore.getDistance(), setpoint));
      }
      else {
        PivotLeftMotor.set(controller.calculate(LeftThroughbore.getDistance(), setpoint));
      }
    }
    else {
      PivotRightMotor.set(controller.calculate(RightThroughbore.getDistance(), setpoint));
      PivotLeftMotor.set(controller.calculate(LeftThroughbore.getDistance(), setpoint));
    } 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right ThroughBore Encoders", RightThroughbore.getDistance());
    SmartDashboard.putNumber("Left ThroughBore Encoders", LeftThroughbore.getDistance());
  }
}
