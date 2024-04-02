// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auton.CommandVariants.IndexPickupAuton;
import frc.robot.commands.Auton.CommandVariants.IndexReturnAuton;
import frc.robot.commands.Auton.CommandVariants.IndexShooterAuton;
import frc.robot.commands.Auton.CommandVariants.IndexStationaryAuton;
import frc.robot.commands.Auton.CommandVariants.IntakeAuton;
import frc.robot.commands.Auton.CommandVariants.IntakeStationaryAuton;
import frc.robot.commands.Auton.CommandVariants.PivotDownAuton;
import frc.robot.commands.Auton.CommandVariants.ShooterPickupAuton;
import frc.robot.commands.Auton.CommandVariants.ShooterShootAuton;
import frc.robot.commands.Auton.CommandVariants.ShooterSpinUpAuton;
import frc.robot.commands.Auton.CommandVariants.ShooterStationaryAuton;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot.PivotTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAuton extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuton. */
  public TwoNoteAuton(Shooter shooter, Indexer indexer, Pivot pivot, Intake intake, Limelight limelight, PathPlannerPath traj1, PathPlannerPath traj2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Timer timer = new Timer();
    timer.reset();
    timer.start();
    addCommands(
      new ParallelCommandGroup(
          new ShooterSpinUpAuton(shooter, timer),
          new SequentialCommandGroup(new WaitCommand(0.75), new IndexShooterAuton(indexer, timer)),
          new PivotDownAuton(pivot, PivotTarget.Intake)),
      new ParallelCommandGroup(
        new IntakeAuton(intake, timer),
        new IndexPickupAuton(indexer, timer),
        new ShooterPickupAuton(shooter, timer),
        AutoBuilder.followPath(traj1)
      ),
      new ParallelCommandGroup(
        new IntakeStationaryAuton(intake, timer),
        new IndexStationaryAuton(indexer, timer),
        new ShooterStationaryAuton(shooter, timer)
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(new IndexReturnAuton(indexer, timer), new ShooterSpinUpAuton(shooter, timer)),
        AutoBuilder.followPath(traj2)
      ),
      new ParallelCommandGroup(
          new ShooterShootAuton(shooter, timer),
          new SequentialCommandGroup(new WaitCommand(0.25), new IndexShooterAuton(indexer, timer))
      )
    );
  }
}
