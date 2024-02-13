// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climber extends SubsystemBase {
  CANSparkMax climber_motor_L, climber_motor_R;
  

  /** Creates a new Climber. */
  public Climber() {
    climber_motor_L = new CANSparkMax(Constants.CLIMBER_MOTOR_1, MotorType.kBrushless);
    climber_motor_R = new CANSparkMax(Constants.CLIMBER_MOTOR_2, MotorType.kBrushless);
  }

  public void runRightClimber(CommandXboxController m_controller) {
    climber_motor_R.set(m_controller.getLeftY());
  }

  public void runLeftClimber(CommandXboxController m_controller) {
    climber_motor_L.set(m_controller.getLeftY());
  }

  public void autoClimb(AHRS m_navx) {
    double theta = m_navx.getRoll();
    double c = Math.sin(theta);

    //Under the assumption that leftward tilt is a positive angle and motor down = positive
    if (theta < -5 || theta > 5) {
      if (c > 0) {
        climber_motor_R.set(c);
      }
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
