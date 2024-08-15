package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

import java.lang.Math; 

public class DriveCommand extends Command {

    private double xDot;
    private double yDot;
    private double thetaDot;
    private boolean fieldRelative;
    private ChassisSpeeds chassisSpeeds, chassisPercent;
    private CommandXboxController m_controller;

    // The subsystem the command runs on
    public final DrivetrainSubsystem drivetrain;
    public final Limelight m_limelight;

    public DriveCommand(DrivetrainSubsystem subsystem, CommandXboxController controller, Limelight limelight){
        drivetrain = subsystem;
        m_controller = controller;
        m_limelight = limelight;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
    }

            
    @Override
    public void execute() {
      if(m_controller.getLeftY() < 0){
        xDot = -(m_controller.getLeftY()*m_controller.getLeftY()*Constants.kMaxTranslationalVelocity);
      }
      else{
        xDot = (m_controller.getLeftY()*m_controller.getLeftY()) * Constants.kMaxTranslationalVelocity;
      }
      if(m_controller.getLeftX() < 0){
        yDot = -(m_controller.getLeftX()*m_controller.getLeftX()*Constants.kMaxTranslationalVelocity);
      }
      else{
        yDot = (m_controller.getLeftX()*m_controller.getLeftX()) * Constants.kMaxTranslationalVelocity;
      }
      if(m_controller.getRightX() < 0){
        thetaDot = -(m_controller.getRightX()*m_controller.getRightX()*Constants.kMaxRotationalVelocity);
      }
      else{
        thetaDot = (m_controller.getRightX()*m_controller.getRightX()) * Constants.kMaxRotationalVelocity;
      }

      // xDot *= 0.05;
      // yDot *= 0.05;
      // thetaDot *= 0.05;

      fieldRelative = true;
      if(Math.abs(xDot)<=0.07*0.07*Constants.kMaxTranslationalVelocity){
        xDot = 0;
      }
      else{
        if(xDot > 0){
          xDot -= (0.07*0.07);
        }
        else{
          xDot += (0.07*0.07);
        }
        xDot *= 1/(1-(0.07*0.07));
      }
      if(Math.abs(yDot)<=0.07*0.07*Constants.kMaxTranslationalVelocity){
        yDot = 0;
      }
      else{
        if(yDot > 0){
          yDot -= (0.07*0.07);
        }
        else{
          yDot += (0.07*0.07);
        }
        yDot *= 1/(1-(0.07*0.07));
      }
      if(Math.abs(thetaDot)<=0.07*0.07*Constants.kMaxRotationalVelocity){
          thetaDot = 0;
      }
      else{
        if(thetaDot > 0){
          thetaDot -= 0.07*0.07;
        }
        else{
          thetaDot += 0.07*0.07;
        }
        thetaDot *= 1/(1-(0.07*0.07));
      }
        
      if(m_controller.leftTrigger(Constants.ROBOT_ORIENTED_DEADBAND).getAsBoolean() && m_controller.rightTrigger(Constants.ROBOT_ORIENTED_DEADBAND).getAsBoolean()){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot*0.25, yDot*0.25, thetaDot*0.25, new Rotation2d(0));

      }
      else if(m_controller.leftTrigger(Constants.ROBOT_ORIENTED_DEADBAND).getAsBoolean()){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot*0.25, yDot*0.25, thetaDot*0.25, new Rotation2d(Math.PI));
      }
      else if(m_controller.rightTrigger(Constants.ROBOT_ORIENTED_DEADBAND).getAsBoolean()){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, new Rotation2d(0));
      }
      else{
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading());
      }
      
      drivetrain.drive(chassisSpeeds, true);
    }
}