package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  double Kp = 1/27;
  NetworkTable limelightTable;
  double ty, tv, tx, angle, distance;
  double min_command = 0.05;
  CommandXboxController xbox1;
  double steering_adjust = 0.0;
  private static Limelight limelight = null;


  public Limelight(CommandXboxController xbox, double inputkP) {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    xbox1 = xbox;
    //Kp = inputkP;
  }

  public double getAngle() {
    SmartDashboard.putNumber("Horizontal Angle:", tx);
    if (tx != 0) {
      return tx;
    } else {
    return 0;
    }
  }


  public double getDistance() {
    if (tv != 0) {
      angle = (Constants.ANGLE_CAMERA + ty) * Math.PI / 180;
      double a = ((Constants.HEIGHT_TARGET - Constants.HEIGHT_CAMERA) / Math.tan(angle)) / 12;
      return a;
    } else {
      return 0;
    }
    
  }

  public static Limelight getInstance(){
    return Limelight.limelight;
  }

  public double getDesiredRPM(){
    double a = 1.3;
    double b = 24.18 + Constants.PIDB_CONSTANT;
    return a*getDistance() + b;
    // return 0.149827*(Math.pow(1.04964, (4.99985*this.getDistance()) + 29.9996) + 28.4836);
    //Add implementation
  }

  @Override
  public void periodic() {
    ty = limelightTable.getEntry("ty").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);
    tx = limelightTable.getEntry("tx").getDouble(0);

    if (xbox1.getRawButton(8) == true)
    {
      double heading_error = -tx;
     
      if (tx > 1.0 || tx < -1.0)
      {
        Drivetrain.getInstance().rightGroup.set(-heading_error/70);
        Drivetrain.getInstance().leftGroup.set(heading_error/70);
      }
    }
    
    SmartDashboard.putNumber("Horizontal", tx);
    SmartDashboard.putNumber("Vertical:", ty);
  }
}