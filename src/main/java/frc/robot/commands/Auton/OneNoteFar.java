// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PIDShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;


public class OneNoteFar extends SequentialCommandGroup {
  /** Creates a new OneNoteFar. */
  public OneNoteFar(Shooter shooter, Indexer indexer, Limelight limelight, PathPlannerPath traj, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new ParallelCommandGroup(
          new PIDShooter(shooter, limelight),
          new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::indexIn))),
        new InstantCommand(shooter::off),
        new InstantCommand(indexer::off),
        new ParallelCommandGroup(
          AutoBuilder.followPath(traj),
          new InstantCommand(pivot::autonPivot),
          new SequentialCommandGroup(new WaitCommand(0.6), new InstantCommand(indexer::indexIn)))
    );
  }
}
