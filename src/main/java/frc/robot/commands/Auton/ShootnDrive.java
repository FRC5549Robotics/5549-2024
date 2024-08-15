// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auton.CommandVariants.IndexShooterAuton;
import frc.robot.commands.Auton.CommandVariants.ShooterSpinUpAuton;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootnDrive extends SequentialCommandGroup {
  /** Creates a new ShootnDrive. */
  public ShootnDrive(Shooter shooter, Indexer indexer, Limelight limelight, PathPlannerPath traj) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Timer timer = new Timer();
    timer.reset();
    timer.start();
    addCommands(
      new ParallelCommandGroup(
          new ShooterSpinUpAuton(shooter, timer),
          new SequentialCommandGroup(new WaitCommand(0.75), new IndexShooterAuton(indexer, timer))),
      new WaitCommand(8),
      AutoBuilder.followPath(traj)
    );
  }
}
