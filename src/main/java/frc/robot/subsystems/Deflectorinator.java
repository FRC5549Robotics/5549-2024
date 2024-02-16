// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Deflectorinator extends SubsystemBase {

  CANSparkMax DeflectMotor;
  PIDController controller;
  RelativeEncoder DeflectEncoder;
  /** Creates a new Deflectorinator. */
  public Deflectorinator() {
    DeflectMotor = new CANSparkMax(Constants.DEFLECTORINATOR_MOTOR, MotorType.kBrushless);
    controller = new PIDController(1.0, 0.0, 0.05);
    DeflectEncoder = DeflectMotor.getEncoder();
    DeflectMotor.setIdleMode(IdleMode.kBrake);
    DeflectEncoder.setPositionConversionFactor(1/Constants.kDeflectGearRatio);
  }

  public void deflectorinate(double speed){
    DeflectMotor.set(speed);
  }
  public void deflectorinateIn(){
    DeflectMotor.set(0.05);
  }
  public void deflectorinateOut(){
    DeflectMotor.set(-0.05);
  }
  public void off(){
    DeflectMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
