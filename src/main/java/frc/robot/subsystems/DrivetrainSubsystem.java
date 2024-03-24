package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import  com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;


@SuppressWarnings("PMD.ExcessiveImports")
public class DrivetrainSubsystem extends SubsystemBase {

  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
          Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
          Constants.FRONT_LEFT_MODULE_STEER_ENCODER
          );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
          Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
          Constants.FRONT_RIGHT_MODULE_STEER_ENCODER
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
        Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        Constants.BACK_LEFT_MODULE_STEER_ENCODER
          );

  private final SwerveModule m_rearRight =
      new SwerveModule(
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER
          );

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
  private double[] lastDistances;
  private double lastTime;
  private double offset = 0.0;
  private final Timer timer;

  // The gyro sensor
  AHRS m_ahrs;
//  private final Gyro m_gyro =  new ADIS16470_IMU(); // new ADXRS450_Gyro();
  // private final PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.kPigeonPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  //target pose and controller
  Pose2d m_targetPose;
  PIDController m_thetaController = new PIDController(1.0, 0.0, 0.05);

  private ChassisSpeeds speeds; 
    
  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem(AHRS ahrs) {
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (ChassisSpeeds speeds) -> 
      drive(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond), true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(0.1, 0.0, 0.0), // Translation PID constants
              new PIDConstants(0.001, 0.0, 0.0), // Rotation PID constants
              4.2, // Max module speed, in m/s
              0.399621397388, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path
        // being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
      
      if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
    m_ahrs = ahrs;
    fixBackRight();

    // Zero out the gyro
    m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics, getHeading(), getModulePositions());

    for (SwerveModule module: modules) {
      module.resetDistance();
      module.syncTurningEncoders();
    }

    m_targetPose = m_odometry.getPoseMeters();
    m_thetaController.reset();
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    lastDistances = new double[]{
      m_frontLeft.getDriveDistanceMeters(),
      m_frontRight.getDriveDistanceMeters(),
      m_rearLeft.getDriveDistanceMeters(),
      m_rearRight.getDriveDistanceMeters(),
    };

    timer = new Timer();
    timer.reset();
    timer.start();
    lastTime = 0;
    new WaitCommand(0.5);
  }

  public void syncEncoders() {
    for (SwerveModule module: modules) {
      module.resetDistance();
      module.syncTurningEncoders();
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry(); 

    SmartDashboard.putNumber("Front Left CANCoder", m_frontLeft.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Front Right CANCoder", m_frontRight.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Back Left CANCoder", m_rearLeft.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);
    SmartDashboard.putNumber("Back Right CANCoder", m_rearRight.getTurnCANcoder().getAbsolutePosition().getValueAsDouble()*360);

    SmartDashboard.putNumber("Front Left Neo Encoder", m_frontLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Front Right Neo Encoder", m_frontRight.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Back Left Neo Encoder", m_rearLeft.getTurnEncoder().getPosition());
    SmartDashboard.putNumber("Back Right Neo Encoder", m_rearRight.getTurnEncoder().getPosition());
    
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());
    SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());

    // This was due to pinion slippage: If it is still happening, uncomment this code
    // if(Math.abs(m_frontRight.getTurnEncoder().getPosition() - m_frontRight.getTurnCANcoderAngle()) > 2){
    //   m_frontRight.getTurnEncoder().setPosition(m_frontRight.getTurnCANcoderAngle());
    // }

    // SmartDashboard.putNumber("Distance 0", modules[0].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 1", modules[1].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 2", modules[2].getDriveDistanceMeters());
    // SmartDashboard.putNumber("Distance 3", modules[3].getDriveDistanceMeters());

    // SmartDashboard.putNumber("Angle 0", modules[0].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 1", modules[1].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 2", modules[2].getTurnCANcoderAngle());
    // SmartDashboard.putNumber("Angle 3", modules[3].getTurnCANcoderAngle());

    
    if (1 <= timer.get() && timer.get() <= 1.5) {
      m_ahrs.zeroYaw();
      System.out.println("Zeroed: " + getHeading());
    }
  }

  public void updateOdometry() {
    double[] distances = new double[]{
      m_frontLeft.getDriveDistanceMeters(),
      m_frontRight.getDriveDistanceMeters(),
      m_rearLeft.getDriveDistanceMeters(),
      m_rearRight.getDriveDistanceMeters(),
    };
    double time = timer.get();
    double dt = time - lastTime;
    lastTime = time;
    if (dt == 0) return;
    m_odometry.update(getHeading(), getModulePositions());
    lastDistances = distances;
  }


  public double getTranlationalVelocity() {
    return Math.hypot(this.speeds.vxMetersPerSecond, this.speeds.vyMetersPerSecond);
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    /* Don't reset all the motors' positions. Otherwise the robot thinks it has teleported!
    for (SwerveModule module: modules) {
      module.resetDistance();
    }
    */
    m_odometry.resetPosition(
      getHeading(), 
      getModulePositions(), 
      pose);
  }

  /**
   * Method to rotate the relative orientation of the target pose at a given rate.
   *
   * @param deltaTheta How much to rotate the target orientation per loop.
   */
  public void rotateRelative(Rotation2d deltaTheta) {
    Transform2d transform = new Transform2d(new Translation2d(), deltaTheta);
    m_targetPose = m_targetPose.transformBy(transform);
  }

  /**
   * Method to set the absolute orientation of the target pose.
   *
   * @param theta The target orientation.
   */
  public void rotateAbsolute(Rotation2d theta) {
    m_targetPose = new Pose2d(new Translation2d(), theta);
  }

  /**
   * Method to get the output of the chassis orientation PID controller.
   *
   */
  public double getThetaDot() {
    double setpoint = m_targetPose.getRotation().getRadians();
    double measurement = getPose().getRotation().getRadians();
    double output = m_thetaController.calculate(measurement, setpoint);
    SmartDashboard.putNumber("PID out", output);
    return output;
  }

  /**
   * Method to drive the robot with given velocities.
   *
   * @param speeds ChassisSpeeds object with the desired chassis speeds [m/s and rad/s].
   */
  @SuppressWarnings("ParameterName")
  public void drive(ChassisSpeeds speeds, boolean normalize) {

    this.speeds = speeds;

    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      brake();
      return;
    }

    SwerveModuleState[] swerveModuleStates =
        Constants.kDriveKinematics.toSwerveModuleStates(speeds);

    // System.out.println(swerveModuleStates[0]+":"+swerveModuleStates[1]+":"+swerveModuleStates[2]+":"+swerveModuleStates[3]);
    if (normalize) normalizeDrive(swerveModuleStates, speeds);
    
    setModuleStates(swerveModuleStates);
  }

  public void openLoopDrive(ChassisSpeeds speeds) {
    this.speeds = speeds;
    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      brake();
      return;
    }

    SwerveModuleState[] swerveModuleStates =
        Constants.kDriveKinematics.toSwerveModuleStates(speeds);
           
    normalizeDrive(swerveModuleStates, speeds);
    
    setModuleStates(swerveModuleStates);
  }

  public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / Constants.kMaxTranslationalVelocity;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / Constants.kMaxRotationalVelocity;
    double k = Math.max(translationalK, rotationalK);

    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : desiredStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
      SmartDashboard.putNumber("Capped Speed", realMaxSpeed);
    }

    double scale = Math.min(k * Constants.kMaxTranslationalVelocity / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : desiredStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }

  public void brake() {
    for (SwerveModule module : modules) {
      module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Preferences.getDouble("kMaxSpeedMetersPerSecond", Constants.kMaxSpeedMetersPerSecond));

        for (int i = 0; i <= 3; i++) {
          modules[i].setDesiredState(desiredStates[i]);
        }
  }

  public void setOpenLoopStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Preferences.getDouble("kMaxSpeedMetersPerSecond", Constants.kMaxSpeedMetersPerSecond));

    for (int i = 0; i <= 3; i++) {
      modules[i].setOpenLoopState(desiredStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
      
    }

    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    for (SwerveModule module: modules) {
      module.resetEncoders();
    }
  }

  // /** Zeroes the heading of the robot. */
  // public void zeroHeading() {
  //   m_ahrs.zeroYaw();
  //   offset = 0;
  //   m_targetPose = new Pose2d(new Translation2d(), new Rotation2d());
  // }

  // public void resetOdometry(double heading, Pose2d pose) {
  //   zeroHeading();
  //   offset = heading;
  //   m_odometry.resetPosition(Rotation2d.fromDegrees(heading),
  //   getModulePositions(),
  //   pose);
  // }
  public void zeroGyroscope(){
    m_ahrs.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    float raw_yaw = -m_ahrs.getYaw(); // Returns yaw as -180 to +180.
    // float raw_yaw = m_ahrs.getYaw(); // Returns yaw as -180 to +180.
    float calc_yaw = raw_yaw;

    if (0.0 > raw_yaw ) { // yaw is negative
      calc_yaw += 360.0;
    }
    return Rotation2d.fromDegrees(calc_yaw);
  }
  

  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      new SwerveModulePosition(m_frontLeft.getDriveDistanceMeters(), m_frontLeft.getState().angle),
      new SwerveModulePosition(m_frontRight.getDriveDistanceMeters(), m_frontRight.getState().angle),
      new SwerveModulePosition(m_rearLeft.getDriveDistanceMeters(), m_rearLeft.getState().angle),
      new SwerveModulePosition(m_rearRight.getDriveDistanceMeters(), m_rearRight.getState().angle)};
  }
  private void fixBackRight(){
    m_rearRight.getTurnMotor().setInverted(false);
  }

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveModuleState[] states = Constants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);

  public Command ChoreoTrajectoryFollower(ChoreoTrajectory traj){
    return Choreo.choreoSwerveCommand(
      traj, 
      this::getPose, 
      new PIDController(5, 0.0, 0.0),  
      new PIDController(5, 0.0, 0.0),  
      new PIDController(0.35, 0.0, 0.0),  
      (ChassisSpeeds speeds) -> 
          drive(new ChassisSpeeds(-speeds.vxMetersPerSecond,-speeds.vyMetersPerSecond,-speeds.omegaRadiansPerSecond), true),
      () -> {
          Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == Alliance.Red;
      },
      this);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

}