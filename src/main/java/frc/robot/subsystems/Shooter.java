// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	private static Shooter instance = null;
	/** Creates a new Shooter. */
	CANSparkFlex ShooterRight, ShooterLeft;
	RelativeEncoder motor1_encoder, motor2_encoder;
	MotorControllerGroup shooterGroup;
	boolean isOn;
	PIDController pid = new PIDController(Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kD);
	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SHOOTER_kV, Constants.SHOOTER_kS, Constants.SHOOTER_kA);
	CommandXboxController Xbox;
	Joystick JoystickRight;
	double targetRPM;


	public Shooter(CommandXboxController xbox, Joystick joystickRight) {
		ShooterRight = new CANSparkFlex(0, MotorType.kBrushless);
		ShooterLeft = new CANSparkFlex(1, MotorType.kBrushless);

		motor1_encoder = ShooterRight.getEncoder();
		motor2_encoder = ShooterLeft.getEncoder();

		SmartDashboard.putNumber("P1 Gain", Constants.SHOOTER_kP);
		SmartDashboard.putNumber("I1 Gain", Constants.SHOOTER_kI);
		SmartDashboard.putNumber("D1 Gain", Constants.SHOOTER_kD);
		Xbox = xbox;
		JoystickRight = joystickRight;
	
		SmartDashboard.putNumber("motor 1 velocity", motor1_encoder.getVelocity());
		SmartDashboard.putNumber("motor 2 velocity", motor2_encoder.getVelocity());
		instance = this;
		targetRPM = 111000;
	}

	public static double getSpeed(double distance) {
		// Add implementation
		return 0.0;
	}

	// public void autonSpeed(){
	// 	shooterGroup.set(Constants.SHOOTER_AUTON_SPEED);
	// }

	public void runShooter(double speed){
		ShooterRight.set(speed);
    ShooterLeft.set(speed);
	}

	public void off(){
		ShooterRight.set(0);
    ShooterLeft.set(0);
	}

	public void on(double setPoint) {
		targetRPM = setPoint;
		SmartDashboard.putNumber("SetPoint", setPoint);
		ShooterRight.setVoltage(pid.calculate(motor1_encoder.getVelocity(), setPoint) + feedforward.calculate(setPoint));
		ShooterLeft.setVoltage(pid.calculate(motor2_encoder.getVelocity(), setPoint) + feedforward.calculate(setPoint));
		// M1pid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
		// M2pid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
		SmartDashboard.putNumber("RPM Left", motor1_encoder.getVelocity());
		SmartDashboard.putNumber("RPM Right", motor2_encoder.getVelocity());
	}
  public void shootAmp(){
		ShooterRight.setVoltage(pid.calculate(motor1_encoder.getVelocity(), Constants.SHOOTER_AMP_VELOCITY) + feedforward.calculate(Constants.SHOOTER_AMP_VELOCITY));
		ShooterLeft.setVoltage(pid.calculate(motor2_encoder.getVelocity(), Constants.SHOOTER_AMP_VELOCITY) + feedforward.calculate(Constants.SHOOTER_AMP_VELOCITY));

  }
	
	@Override
	public void periodic(){
	}

	public Shooter getInstance() {
		return instance;
	}
}