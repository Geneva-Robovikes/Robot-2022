// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class LaunchCommand extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LaunchCommand(LaunchSubsystem subsystem) {
    launchSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setLaunchMotors(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setLaunchMotors(0);
  }
}
