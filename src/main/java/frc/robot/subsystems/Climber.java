// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase {
  CANSparkMax climber_motor_L, climber_motor_R;

  /** Creates a new Climber. */
  public Climber() {
    climber_motor_L = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    climber_motor_R = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    { // CAN Status Frames
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 400);
      climber_motor_L.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 400);
      climber_motor_R.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 400);

    }
  }

  public void runRightClimber(double speed) {
    climber_motor_R.set(speed);
  }

  public void runLeftClimber(double speed) {
    climber_motor_L.set(speed);
  }

  public void autoClimb(AHRS m_navx) {
    double theta = m_navx.getRoll();
    double c = Math.sin(Math.toRadians(theta));

    //Under the assumption that leftward tilt is a positive angle and motor down = positive
    if (theta < -3 || theta > 3) {
      if (c > 0) {
        climber_motor_R.set(c);
      }
      else {
        climber_motor_L.set(-c);
      }
    }
    else{
      if(climber_motor_L.getEncoder().getPosition() < Constants.CLIMBER_LEFT_ENCODER_MAX && climber_motor_R.getEncoder().getPosition() > Constants.CLIMBER_RIGHT_ENCODER_MAX) {
        climber_motor_L.set(Constants.CLIMBER_SCALING_FACTOR);
        climber_motor_R.set(Constants.CLIMBER_SCALING_FACTOR);
      }
      else{
        climber_motor_L.set(0);
        climber_motor_R.set(0);
      }
    }
  }

  public void off() {
    climber_motor_L.set(0);
    climber_motor_R.set(0);
  }

  public void leftOff() {
    climber_motor_L.set(0);
  }  

  public void rightOff() {
    climber_motor_R.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
