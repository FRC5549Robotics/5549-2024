// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  CANSparkMax IndexerMotor;
  AnalogInput analog;
  ColorSensorV3 m_colorSensor;
  I2C.Port i2cPort;
  ColorMatch m_colorMatcher;
  Color kOrangeTarget = new Color(0.388, 0.2, 0.012);
  Color kBlack = new Color(0, 0, 0);
  Color detectedColor;
  ColorMatchResult match;
  

  AddressableLED LED;
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.INDEXER_LED_STRIP_LENGTH);
  Color kGreen1 = new Color(20, 150,  0);
  Color kOrange1 = new Color(255, 25, 0);

  public Indexer(AddressableLED led) {
    IndexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR, MotorType.kBrushless);
    IndexerMotor.setIdleMode(IdleMode.kCoast);
    analog = new AnalogInput(0);

    i2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2cPort);
    m_colorMatcher = new ColorMatch();
    m_colorMatcher.addColorMatch(kOrangeTarget);

    LED = led;
    LED.setLength(ledBuffer.getLength());
    LED.setData(ledBuffer);
    LED.start();
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, kGreen1);
    }
    LED.setData(ledBuffer);
  }

  public void indexIn(){
    IndexerMotor.set(-Constants.INDEXER_SPEED);
    // if (analog.getVoltage() < Constants.SENSOR_VOLTAGE_THRESHOLD) {
    //   IndexerMotor.set(-Constants.INDEXER_SPEED);
    //   return false;
    // }
    // else {
    //   IndexerMotor.set(0);
    //   for(int i = 0; i < ledBuffer.getLength(); i++){
    //     ledBuffer.setLED(i, kOrange1);
    //   }
    //   LED.setData(ledBuffer);
    //   return true;
    // }
    // if (match.color != kOrangeTarget){
    // IndexerMotor.set(-Constants.INDEXER_SPEED);
    // } 
    // else {
    //   IndexerMotor.set(0);
    //   for(int i = 0; i < ledBuffer.getLength(); i++){
    //     ledBuffer.setLED(i, kOrange1);
    //   }
    // LED.setData(ledBuffer);
    // }    
  }
  public void indexOut(){
    IndexerMotor.set(Constants.INDEXER_SPEED_OUT);
  }

  public void off(){
    IndexerMotor.set(0);
    resetLEDs();
  }

  public void resetLEDs(){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setLED(i, kGreen1);
    }
    LED.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Output", analog.getVoltage());
    SmartDashboard.putNumber("Color Sensor IR", m_colorSensor.getIR());
    SmartDashboard.putNumber("Color Sensor Red", m_colorSensor.getColor().red);
    SmartDashboard.putNumber("Color Sensor Green", m_colorSensor.getColor().green);    
    SmartDashboard.putNumber("Color Sensor Blue", m_colorSensor.getColor().blue);
    SmartDashboard.putString("Color Matched", m_colorSensor.getColor().toString());
    // This method will be called once per scheduler run
  }
}
