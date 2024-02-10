// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake;

// public class IntakeAnalog extends Command {
//   /** Creates a new Intake. */
//   Intake m_intake;
//   CommandXboxController m_controller;
//   public IntakeAnalog(Intake intake, CommandXboxController controller) {
//     m_intake = intake;
//     m_controller = controller;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_intake.intake(m_controller.getLeftTriggerAxis()*Constants.INTAKE_SCALING_FACTOR);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
