// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbCommandDown extends CommandBase {
  private final ClimbSubsystem climbSubsystem;
  private double maxHeight = 307200;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommandDown(ClimbSubsystem subsystem) {
    climbSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.setClimbMotors(-0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(climbSubsystem.getRightClimbEncoder() > maxHeight) {return true;}
    else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setClimbMotors(0);
  }
}
