// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class TurnPIDCommand extends PIDCommand {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnPIDCommand(DriveSubsystem drive, double angle) {
    super (
      new PIDController(Constants.kPDriveVel, 0, 0),
      drive::getHeading,
      angle,
      output -> drive.arcadeDrive(0, output),
      drive);

    getController().enableContinuousInput(-180, 180);
    //use this if things are a bit spicy
    //getController().setTolerance(positionTolerance, velocityTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
