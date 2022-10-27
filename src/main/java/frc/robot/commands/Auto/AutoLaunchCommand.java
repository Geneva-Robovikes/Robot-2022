// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class AutoLaunchCommand extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private double launchSpeed;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoLaunchCommand(LaunchSubsystem subsystem, double speed) {
    launchSubsystem = subsystem;
    launchSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchSubsystem.setLaunchMotors(launchSpeed);
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
