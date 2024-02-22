package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

      xDot = m_controller.getLeftY() * Constants.kMaxTranslationalVelocity;
      yDot = m_controller.getLeftX() * Constants.kMaxTranslationalVelocity;
      if (m_controller.getHID().getRightTriggerAxis() > 0.1) {
        thetaDot = m_limelight.getSpeakerTheta();
        if (-2 > m_limelight.getAngle() || 2 < m_limelight.getAngle()) {
          thetaDot = 0;
        }
      }
      else if (m_controller.getHID().getLeftTriggerAxis() > 0.1) {
        thetaDot = m_limelight.getAmpTheta();
        if (-2 > m_limelight.getAngle() || 2 < m_limelight.getAngle()) {
          thetaDot = 0;
        }
      }
      else{
        thetaDot = m_controller.getRightX() * Constants.kMaxRotationalVelocity;
      }
      fieldRelative = true;
      if(Math.abs(xDot)<=0.06*Constants.kMaxTranslationalVelocity){
        xDot = 0;
      }
      else{
        xDot -= 0.06;
        xDot *= 1/0.96;
      }
      if(Math.abs(yDot)<=0.06*Constants.kMaxTranslationalVelocity){
        yDot = 0;
      }
      else{
        yDot -= 0.06;
        yDot *= 1/0.96;
      }
      if(Math.abs(thetaDot)<=0.06*Constants.kMaxRotationalVelocity){
          thetaDot = 0;
      }
      else{
        thetaDot -= 0.06;
        thetaDot *= 1/0.96;
      }
        
        

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xDot, yDot, thetaDot, drivetrain.getHeading());
      //System.out.println(drivetrain.getHeading());
      if(chassisSpeeds == new ChassisSpeeds(xDot, yDot, thetaDot)){
        System.out.println("good");
      }
      System.out.println(xDot+":"+yDot+":"+thetaDot);
      
      drivetrain.drive(chassisSpeeds, true);
    }
}