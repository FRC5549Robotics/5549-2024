// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.ClimberAnalog;
import frc.robot.commands.DeflectRotate;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PIDShooter;
import frc.robot.commands.PivotAnalog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.Auton.OneNoteAutonNoDrive;
import frc.robot.commands.Auton.ShootnDrive;
import frc.robot.commands.Auton.SimpleDrive;
import frc.robot.commands.Auton.TwoNoteAuton;
import frc.robot.commands.ClimberAnalog.Side;
import frc.robot.commands.IntakeAnalog;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.SPI;
import org.ejml.simple.SimpleBase;
import org.ejml.simple.SimpleMatrix;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Deflectorinator;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotTarget;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final CommandXboxController m_controller = new CommandXboxController(Constants.DRIVE_CONTROLLER);
  private final CommandXboxController m_controller2 = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final AHRS m_ahrs = new AHRS();
  public final DrivetrainSubsystem m_drive = new DrivetrainSubsystem(m_ahrs);
  private final Pivot m_pivot = new Pivot(m_controller2);
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Deflectorinator m_deflectorinator = new Deflectorinator();
  private final Climber m_climber = new Climber();
  private final Limelight m_limelight = new Limelight();//Find Feedforward Constants );
  private final AddressableLED led = new AddressableLED(9);
  private final Indexer m_indexer = new Indexer(led);



  JoystickButton resetNavXButton = new JoystickButton(m_controller.getHID(), Constants.RESET_NAVX_BUTTON);
  JoystickButton deployPivotButton = new JoystickButton(m_controller2.getHID(), Constants.DEPLOY_PIVOT_BUTTON);
  JoystickButton retractPivotButton = new JoystickButton(m_controller2.getHID(), Constants.RETRACT_PIVOT_BUTTON);
  JoystickButton intakeShootingButton = new JoystickButton(m_controller2.getHID(), Constants.INTAKE_SHOOTER_BUTTON);
  JoystickButton autoClimbButton = new JoystickButton(m_controller.getHID(), Constants.CLIMBER_BUTTON);
  JoystickButton indexerInButton = new JoystickButton(m_controller2.getHID(), Constants.INDEXER_IN_BUTTON);
  JoystickButton indexerOutButton = new JoystickButton(m_controller2.getHID(), Constants.INDEXER_OUT_BUTTON);
  JoystickButton shooterIntakingButton = new JoystickButton(m_controller2.getHID(), Constants.SHOOTER_INTAKE_BUTTON);
  JoystickButton shooterAmpButton = new JoystickButton(m_controller2.getHID(), Constants.SHOOTER_AMP_BUTTON);
  JoystickButton deflectorinatorInButton = new JoystickButton(m_controller.getHID(), Constants.DEFLECTORINATOR_IN_BUTTON);
  JoystickButton deflectorinatorOutButton = new JoystickButton(m_controller.getHID(), Constants.DEFLECTORINATOR_OUT_BUTTON);
 // JoystickButton ampPivotButton = new JoystickButton(m_controller2.getHID(), Constants.AMP_SHOOTER_BUTTON);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  // public static ChoreoTrajectory LefttoNote = Choreo.getTrajectory("LefttoNote");
  // public static ChoreoTrajectory NotetoLeft = Choreo.getTrajectory("NotetoLeft");
  // public static ChoreoTrajectory MidtoNote = Choreo.getTrajectory("MidtoNote");
  // public static ChoreoTrajectory NotetoMid = Choreo.getTrajectory("NotetoMid");
  // public static ChoreoTrajectory RighttoNote = Choreo.getTrajectory("RighttoNote");
  // public static ChoreoTrajectory NotetoRight = Choreo.getTrajectory("NotetoRight");

  // PathPlannerPath MidtoNoteM = PathPlannerPath.fromPathFile("MidtoNoteM");
  // PathPlannerPath NoteMtoMid = PathPlannerPath.fromPathFile("NoteMtoMid");
  // PathPlannerPath MidtoNoteR = PathPlannerPath.fromPathFile("MidtoNoteR");
  // PathPlannerPath NoteRtoMid = PathPlannerPath.fromPathFile("NoteRtoMid");
  // PathPlannerPath MidtoNoteL = PathPlannerPath.fromPathFile("MidtoNoteL");
  // PathPlannerPath NoteLtoMid = PathPlannerPath.fromPathFile("NoteLtoMid");
  // PathPlannerPath RighttoNoteR = PathPlannerPath.fromPathFile("RighttoNoteR");
  // PathPlannerPath NoteRtoRight = PathPlannerPath.fromPathFile("NoteRtoRight");
  // PathPlannerPath RighttoNoteM = PathPlannerPath.fromPathFile("RighttoNoteM");
  // PathPlannerPath NoteMtoRight = PathPlannerPath.fromPathFile("NoteMtoRight");
  // PathPlannerPath LefttoNoteL = PathPlannerPath.fromPathFile("LefttoNoteL");
  // PathPlannerPath NoteLtoLeft = PathPlannerPath.fromPathFile("NoteLtoLeft");
  // PathPlannerPath LefttoNoteM = PathPlannerPath.fromPathFile("LefttoNoteM");
  // PathPlannerPath NoteMtoLeft = PathPlannerPath.fromPathFile("NoteMtoLeft");
  // PathPlannerPath t3 = PathPlannerPath.fromPathFile("Simple2");
  // PathPlannerPath t4 = PathPlannerPath.fromPathFile("Simple3");
  // PathPlannerPath LeftMove = PathPlannerPath.fromPathFile("LeftMove");
  // PathPlannerPath RightMove = PathPlannerPath.fromPathFile("RightMove");

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //m_autoChooser.setDefaultOption("Only Drive Middle", m_ZeroConeAutoMiddle);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      m_drive.setDefaultCommand(new DriveCommand(m_drive, m_controller, m_limelight));
      resetNavXButton.onTrue(new InstantCommand(m_drive::zeroGyroscope));

    // Pivot
     deployPivotButton.whileTrue(new PivotIntake(m_pivot, PivotTarget.Intake));
     retractPivotButton.whileTrue(new PivotIntake(m_pivot, PivotTarget.Retracted));
    //  ampPivotButton.whileTrue(new PivotIntake(m_pivot, PivotTarget.Amp));
     deployPivotButton.or(retractPivotButton).onFalse(new InstantCommand(m_pivot::off));
    //  m_controller2.axisGreaterThan(Constants.PIVOT_JOYSTICK, Constants.PIVOT_DEADBAND).or(m_controller2.axisLessThan(Constants.PIVOT_JOYSTICK, -Constants.PIVOT_DEADBAND)).onTrue(new PivotAnalog(m_pivot, m_controller2)).onFalse(new InstantCommand(m_pivot::off));
    
    // Intake
     m_controller2.axisGreaterThan(Constants.INTAKE_TRIGGER, Constants.INTAKE_DEADBAND).whileTrue(new IntakeAnalog(m_intake, m_controller2));
     intakeShootingButton.onTrue(new InstantCommand(m_intake::shoot)).onFalse(new InstantCommand(m_intake::off));
    //  m_controller2.axisGreaterThan(Constants.INTAKE_TRIGGER, Constants.INTAKE_DEADBAND).onFalse(new InstantCommand(m_intake::off));
    
    // Indexer
     m_controller2.axisGreaterThan(Constants.INTAKE_TRIGGER, Constants.INTAKE_DEADBAND).onTrue(new InstantCommand(m_indexer::indexIn));
     m_controller2.axisGreaterThan(Constants.INTAKE_TRIGGER, Constants.INTAKE_DEADBAND).onFalse(new InstantCommand(m_indexer::off));
    intakeShootingButton.onTrue(new InstantCommand(m_indexer::indexOut)).onFalse(new InstantCommand(m_indexer::off));

    // Shooter
    //Change this line to include parallel command group: PIDShooter, sequentialCommandGroup(wait, index) 
     m_controller2.axisGreaterThan(Constants.SHOOTER_TRIGGER, Constants.SHOOTER_TRIGGER_THRESHOLD).whileTrue(new PIDShooter(m_shooter, m_limelight)).whileFalse(new InstantCommand(m_shooter::off));
     shooterIntakingButton.onTrue(new InstantCommand(m_shooter::shooterIn)).onFalse(new InstantCommand(m_shooter::off));
     shooterAmpButton.onTrue(new InstantCommand(m_shooter::shooterAmp)).onFalse(new InstantCommand(m_shooter::off));

    // Deflectorinator
     deflectorinatorInButton.whileTrue(new InstantCommand(m_deflectorinator::deflectorinateIn));
     deflectorinatorOutButton.whileTrue(new InstantCommand(m_deflectorinator::deflectorinateOut));
     deflectorinatorInButton.or(deflectorinatorOutButton).onFalse(new InstantCommand(m_deflectorinator::off));
    //  deflectorinatorOutButton.whileTrue(new DeflectRotate(m_deflectorinator, Deflectorinator.DeflectorinatorTarget.AmpShooting));
    //  deflectorinatorInButton.whileTrue(new DeflectRotate(m_deflectorinator, Deflectorinator.DeflectorinatorTarget.Retracted));
    //  deflectorinatorInButton.or(deflectorinatorOutButton).onFalse(new InstantCommand(m_deflectorinator::off));

    // Climber
    //  autoClimbButton.onTrue(new AutoClimb(m_climber, m_ahrs)).onFalse(new InstantCommand(m_climber::off));
     m_controller2.axisGreaterThan(Constants.CLIMBER_LEFT_JOYSTICK, Constants.CLIMBER_DEADBAND).or(m_controller2.axisLessThan(Constants.CLIMBER_LEFT_JOYSTICK, -Constants.CLIMBER_DEADBAND)).whileTrue(new ClimberAnalog(m_climber, m_controller2, Side.Left)).onFalse(new InstantCommand(m_climber::leftOff));
     m_controller2.axisGreaterThan(Constants.CLIMBER_RIGHT_JOYSTICK, Constants.CLIMBER_DEADBAND).or(m_controller2.axisLessThan(Constants.CLIMBER_RIGHT_JOYSTICK, -Constants.CLIMBER_DEADBAND)).whileTrue(new ClimberAnalog(m_climber, m_controller2, Side.Right)).onFalse(new InstantCommand(m_climber::rightOff));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomou

    // m_drive.resetOdometry(new Pose2d(new Translation2d(1.75, 5.5), new Rotation2d(0)));
    // m_drive.resetOdometry(RightMove.getPreviewStartingHolonomicPose());
    
    // return AutoBuilder.followPath(LeftMove);
    // return new ShootnDrive(m_shooter, m_indexer, m_limelight, RightMove);
    // return new TwoNoteAuton(m_shooter, m_indexer, m_pivot, m_intake, m_limelight, MidtoNoteM, NoteMtoMid);
    // return AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(0.3, 0), new Rotation2d(0)), new PathConstraints(0.5, 0.5, 0.5, 0.5));
    // return new SequentialCommandGroup(m_drive.ChoreoTrajectoryFoll[ower(traj), new InstantCommand(m_drive::ChoreoTest));
    // return new OneNoteAutonNoDrive(m_shooter, m_indexer, m_limelight);
    // return new SimpleDrive(m_drive);
    return null;
  }
}

