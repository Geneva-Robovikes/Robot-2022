// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeltSubsystem;

/** An example command that uses an example subsystem. */
public class AutoBeltCommand extends CommandBase {
  private final BeltSubsystem beltSubsystem;
  private final XboxController controller;
  private Timer timer = new Timer();
  private final double beltSpeed;
  private double rumbleLength = 0.5;
  private boolean canRumble = true;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoBeltCommand(BeltSubsystem subsystem, double speed, XboxController xboxController) {
    beltSubsystem = subsystem;
    beltSpeed = speed;
    controller = xboxController;
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
    if(beltSubsystem.getSwitch1State() || beltSubsystem.getSwitch2State()) {
      beltSubsystem.setBeltMotor(0);
      if(canRumble = true) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        canRumble = false;
        if(timer.get() > rumbleLength) {
          canRumble = false;
        }
        timer.start();
      } else {
        timer.stop();
      }
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
