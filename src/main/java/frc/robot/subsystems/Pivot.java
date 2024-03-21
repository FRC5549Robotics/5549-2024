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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

  
public class Pivot extends SubsystemBase {

  public enum PivotTarget{
    Retracted,
    Intake,
    Amp
  }

  CANSparkMax PivotRightMotor;
  CANSparkMax PivotLeftMotor;
  PIDController controllerRight, controllerLeft;
  DutyCycleEncoder RightThroughbore;
  DutyCycleEncoder LeftThroughbore;
  CommandXboxController XboxController;
  boolean intakePosition = false;

  /** Creates a new Pivot. */
  public Pivot(CommandXboxController xboxController) {
    XboxController = xboxController;
    PivotRightMotor = new CANSparkMax(Constants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    PivotLeftMotor = new CANSparkMax(Constants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    controllerRight = new PIDController(0.007, 0.0, 0.00);
    controllerLeft = new PIDController(0.007, 0.0, 0.00);
    RightThroughbore = new DutyCycleEncoder(Constants.PIVOT_ENCODER_RIGHT);
    LeftThroughbore = new DutyCycleEncoder(Constants.PIVOT_ENCODER_LEFT);
    RightThroughbore.setPositionOffset(Constants.PIVOT_OFFSET_RIGHT);
    LeftThroughbore.setPositionOffset(Constants.PIVOT_OFFSET_LEFT);
    RightThroughbore.setDistancePerRotation(360);
    LeftThroughbore.setDistancePerRotation(360);
    PivotLeftMotor.setIdleMode(IdleMode.kBrake);
    PivotRightMotor.setIdleMode(IdleMode.kBrake);
    if(LeftThroughbore.getDistance()>0){
      intakePosition = false;
    }
  }

  public void pivot(double speed){
  
    PivotRightMotor.set(speed);
    PivotLeftMotor.set(-speed);

  }
  public void off(){
    PivotRightMotor.set(0);
    PivotLeftMotor.set(0);
  }


  public void autonPivot() {
    PivotRightMotor.set(-controllerRight.calculate(RightThroughbore.getDistance(), Constants.PIVOT_RIGHT_INTAKE_SETPOINT));
    PivotLeftMotor.set(-controllerLeft.calculate(LeftThroughbore.getDistance(), Constants.PIVOT_LEFT_INTAKE_SETPOINT));
  }

  public void autonPivotIn() {
    PivotRightMotor.set(-controllerRight.calculate(RightThroughbore.getDistance(), Constants.PIVOT_RIGHT_RETRACTED_SETPOINT));
    PivotLeftMotor.set(-controllerLeft.calculate(LeftThroughbore.getDistance(), Constants.PIVOT_LEFT_RETRACTED_SETPOINT));
  }
  
  public void checkLag(double leftSetpoint, double rightSetpoint) {
    // if(Math.abs(RightThroughbore.getDistance() - rightSetpoint) > 2){
    //   System.out.println("run right");
    //   PivotRightMotor.set(controller.calculate(RightThroughbore.getDistance(), rightSetpoint));
    // }
    // else{ 
    //   System.out.println("stop right");
    //   PivotRightMotor.set(0);
    // }
    // if(Math.abs(LeftThroughbore.getDistance() - leftSetpoint) > 2){
    //   System.out.println("run left");
    //   PivotLeftMotor.set(controller.calculate(LeftThroughbore.getDistance(), leftSetpoint));
    // }
    // else{
    //   System.out.println("dtop left");
    //   PivotLeftMotor.set(0);
    // }
    PivotRightMotor.set(-controllerRight.calculate(RightThroughbore.getDistance(), rightSetpoint));
    PivotLeftMotor.set(-controllerLeft.calculate(LeftThroughbore.getDistance(), leftSetpoint));
    System.out.println("Running");
    SmartDashboard.putNumber("rs", rightSetpoint);
    SmartDashboard.putNumber("ls", leftSetpoint);
  }
  public double getRightPosition(){
    return RightThroughbore.getDistance();
  }
  public double getLeftPosition(){
    return LeftThroughbore.getDistance();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right ThroughBore Encoders", RightThroughbore.getDistance());
    
    SmartDashboard.putNumber("Left ThroughBore Encoders", LeftThroughbore.getDistance());
    
    if(!XboxController.a().getAsBoolean() && !XboxController.x().getAsBoolean() && intakePosition){
    PivotRightMotor.set(-controllerRight.calculate(RightThroughbore.getDistance(), Constants.PIVOT_RIGHT_RETRACTED_SETPOINT));
    PivotLeftMotor.set(-controllerLeft.calculate(LeftThroughbore.getDistance(), Constants.PIVOT_LEFT_RETRACTED_SETPOINT));
    }
  }
}
