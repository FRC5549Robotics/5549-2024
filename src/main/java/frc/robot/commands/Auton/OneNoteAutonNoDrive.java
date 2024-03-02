// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeAnalog;
import frc.robot.commands.PIDShooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAutonNoDrive extends SequentialCommandGroup {
  /** Creates a new OneNoteAuton. */
  public OneNoteAutonNoDrive(Shooter shooter, Indexer indexer, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
          new PIDShooter(shooter, limelight),
          new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::indexIn))),
        new WaitCommand(1),
        new InstantCommand(shooter::off),
        new InstantCommand(indexer::off)
    );
  }
}
