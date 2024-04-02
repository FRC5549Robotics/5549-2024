// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflectorinator extends SubsystemBase {

  public enum DeflectorinatorTarget{
    Retracted,
    AmpShooting,
  }

  CANSparkMax DeflectMotor;
  CANSparkLowLevel deflect;
  PIDController controller;
  RelativeEncoder DeflectEncoder;
  /** Creates a new Deflectorinator. */
  public Deflectorinator() {
    DeflectMotor = new CANSparkMax(Constants.DEFLECTORINATOR_MOTOR, MotorType.kBrushless);
    controller = new PIDController(1, 0.0, 0.0);
    DeflectEncoder = DeflectMotor.getEncoder();
    DeflectMotor.setIdleMode(IdleMode.kBrake);
    DeflectEncoder.setPositionConversionFactor(1/Constants.kDeflectGearRatio);
    { //CAN Status Frames
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 400);
      DeflectMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 400);
    }
  }

  public void deflectorinate(double speed){
    DeflectMotor.set(speed);
  }
  public void deflectorinateIn(){
    DeflectMotor.set(-Constants.DEFLECTORINATOR_SPEED);
  }
  public void deflectorinateOut(){
    DeflectMotor.set(Constants.DEFLECTORINATOR_SPEED);
  }
  public void off(){
    DeflectMotor.set(0);
  }

  public void deflectorinatorEncoderPivot(double setpoint) {
    DeflectMotor.set(controller.calculate(DeflectEncoder.getPosition(), setpoint));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Deflectorinator Position", DeflectEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
