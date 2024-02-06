// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax IntakeMotor;
  public Intake() {
    IntakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
  }
  public void intake(double speed) {//make dependent on trigger
    IntakeMotor.set(-speed);
    System.out.println("kind of works");
  }

  public void shoot() {
    IntakeMotor.set(-Constants.INTAKE_OUTTAKE_SPEED);
  }

  public void off(){
    IntakeMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
