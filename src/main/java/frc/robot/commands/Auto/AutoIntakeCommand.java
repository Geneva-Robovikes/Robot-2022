// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class AutoIntakeCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private Timer timer = new Timer();
  private double waitTime = 1.5;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoIntakeCommand(IntakeSubsystem subsystem) {
    intakeSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setRollerMotor(.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > waitTime) {
      intakeSubsystem.setRollerMotor(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setRollerMotor(0);
  }
}
