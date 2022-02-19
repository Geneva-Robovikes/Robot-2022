// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private XboxController xboxController;
  private double halfSpeed = 2;
  private double fullSpeed = 1;
  private double threeQuarters = (1.33333);
  //private double controllerScaleR = (1/.53);
  private double deadzoneX = 0.1;
  private double deadzoneY = 0.1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrive(DriveSubsystem subsystem, XboxController controller) {
    xboxController = controller;
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xboxController.getLeftX();
    double y = xboxController.getLeftY();
    //System.out.println("x: " + x + ", y: " + y);
    if((x > deadzoneX || x < -deadzoneX) || (y > deadzoneY || y < -deadzoneY)){
      //driveSubsystem.curvatureDrive(-y / controllerScaleL, x / 3);
      driveSubsystem.arcadeDrive(-y/threeQuarters, x/threeQuarters);
      //System.out.println("Driving!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
