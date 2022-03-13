// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private XboxController xboxController;
  private double halfSpeed = 2;
  private double threeQuarters = 1.33333;
  private double[] driveSpeedList = new double[2];
  private double[] launchSpeedList = new double[2];
  private int rightIndex = 0;
  //private double inBetween = (1.6);
  //private double controllerScaleR = (1/.53);
  private double deadzoneX = 0.5;
  private double deadzoneY = 0.5;
  private double changeDriveSpeed = 2;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrive(DriveSubsystem subsystem, XboxController controller) {
    xboxController = controller;
    driveSubsystem = subsystem;
    driveSpeedList[0] = halfSpeed;
    driveSpeedList[1] = threeQuarters;
    launchSpeedList[0] = 0.5;
    launchSpeedList[1] = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xboxController.getRightX();
    double y = xboxController.getLeftY();
    boolean rightBumperPressed = xboxController.getRightBumperPressed();
    
    if (rightBumperPressed) {
      rightIndex++;
      if(rightIndex > driveSpeedList.length - 1) {
        rightIndex = 0;
      }
      SmartDashboard.putNumber("Drive Speed", rightIndex + 1);
      changeDriveSpeed = driveSpeedList[rightIndex];
    }

    if((x > deadzoneX || x < -deadzoneX) || (y > deadzoneY || y < -deadzoneY)){
      driveSubsystem.arcadeDrive(-y/changeDriveSpeed, x/changeDriveSpeed);
    } else {
      driveSubsystem.arcadeDrive(0, 0);
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
