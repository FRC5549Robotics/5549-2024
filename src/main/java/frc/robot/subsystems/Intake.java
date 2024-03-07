// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkFlex IntakeMotorBottom;
  CANSparkMax IntakeMotorTop;

  public Intake(){
    IntakeMotorBottom = new CANSparkFlex(Constants.INTAKE_MOTOR_BOTTOM, MotorType.kBrushless);
    IntakeMotorTop = new CANSparkMax(Constants.INTAKE_MOTOR_TOP, MotorType.kBrushless);
    IntakeMotorBottom.setIdleMode(IdleMode.kCoast);
    IntakeMotorTop.setIdleMode(IdleMode.kCoast);
  }

  public void intake(double speed) {//make dependent on trigger
    IntakeMotorBottom.set(-speed);
    IntakeMotorTop.set(-speed);
    System.out.println("kind of works");
  }

  public void 
  shoot() {
    IntakeMotorBottom.set(Constants.INTAKE_OUTTAKE_SPEED);
    IntakeMotorTop.set(Constants.INTAKE_OUTTAKE_SPEED);
  }

  public void off(){
    IntakeMotorBottom.set(0);
    IntakeMotorTop.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
