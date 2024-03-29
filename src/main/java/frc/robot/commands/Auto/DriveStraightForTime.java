// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveStraightForTime extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double lspeed = 0.4;
  //private double rspeed = 0.43;
  private double time;
  private Timer timer = new Timer();
  //private double speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightForTime(DriveSubsystem subsystem, double speed, double time) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.time = time;
    lspeed = speed;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.tankDrive(lspeed, lspeed);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDrive(0, 0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() > time) {return true;}
    else {return false;}
  }
}
