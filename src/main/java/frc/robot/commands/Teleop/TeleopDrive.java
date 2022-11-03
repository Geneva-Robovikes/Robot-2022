// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private XboxController xboxController;
  private double[] driveSpeedList = new double[2];
  private int rightIndex = 0;
  PIDController leftPidController = new PIDController(Constants.kPDriveVel, 0, 0);

  public TeleopDrive(DriveSubsystem driveSubsystem, XboxController xboxController) {
    this.driveSubsystem = driveSubsystem;
    this.xboxController = xboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSpeedList[0] = 4;
    driveSpeedList[1] = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = xboxController.getRightX();
    double y = xboxController.getLeftY();
    boolean rightStickPressed = xboxController.getLeftStickButtonPressed();
    SmartDashboard.putNumber("Drive Speed", rightIndex + 1);

    if (rightStickPressed) {
      rightIndex++;
      if(rightIndex > driveSpeedList.length - 1) {
        rightIndex = 0;
      }
    }


    driveSubsystem.arcadeDrive(x, -y);
    /*joystick deadzone
    if((x > deadzoneX || x < -deadzoneX) || (y > deadzoneY || y < -deadzoneY)){
      driveSubsystem.arcadeDrive(-y/driveSpeedList[rightIndex], x/driveSpeedList[rightIndex]);
      driveSubsystem.setZeroAngle(driveSubsystem.getGyro());
    } else {
      driveSubsystem.arcadeDrive(0, 0);
    }*/
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
