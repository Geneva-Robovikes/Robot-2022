// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsystem;

/** An example command that uses an example subsystem. */
public class AutoBeltCommand extends CommandBase {
  private final BeltSubsystem beltSubsystem;
  private final double beltSpeed;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoBeltCommand(BeltSubsystem subsystem, double speed) {
    beltSubsystem = subsystem;
    beltSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beltSubsystem.setBeltMotor(beltSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //only other working limit switch is on by default, will fix later :/
    if(beltSubsystem.getSwitch1State() || !beltSubsystem.getSwitch2State()) {
      beltSubsystem.setBeltMotor(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    beltSubsystem.setBeltMotor(0);
  }
}
