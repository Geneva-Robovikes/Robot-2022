// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class DriveStraightPIDCommand extends PIDCommand {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightPIDCommand(DriveSubsystem drive, double distance) {
    super (
      new PIDController(Constants.kPDriveVel, 0, 0),
      drive::getLeftEncoderMeters,
      drive.getLeftEncoderMeters() + distance,
      output -> drive.tankDrive(output, output),
      drive);

    getController().setTolerance(0.1, 0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
