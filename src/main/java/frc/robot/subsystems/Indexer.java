// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  CANSparkMax IndexerMotor;
  AnalogInput analog;
  public Indexer() {
    IndexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR, MotorType.kBrushless);
    IndexerMotor.setIdleMode(IdleMode.kCoast);
    analog = new AnalogInput(0);
  }
  public void indexIn(){
    // if (analog.getVoltage() < Constants.SENSOR_VOLTAGE_THRESHOLD) {
    //   IndexerMotor.set(-Constants.INDEXER_SPEED);
    // }
    // else {
    //   IndexerMotor.set(0);
    // }
    IndexerMotor.set(-Constants.INDEXER_SPEED);
    
  }
    public void indexOut(){
    IndexerMotor.set(Constants.INDEXER_SPEED);
  }
  public void off(){
    IndexerMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Output", analog.getVoltage());
    // This method will be called once per scheduler run
  }
}
