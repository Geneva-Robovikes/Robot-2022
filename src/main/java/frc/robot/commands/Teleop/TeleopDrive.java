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
  private double halfSpeed = 1.6;
  private double threeQuarters = 1.33333;
  private double[] driveSpeedList = new double[2];
  private double[] launchSpeedList = new double[2];
  private int rightIndex = 0;
  //private double inBetween = (1.6);
  //private double controllerScaleR = (1/.53);
  private double deadzoneX = 0.4;
  private double deadzoneY = 0.4;
  private double changeDriveSpeed = 2;
  private double previousX;
  private double previousY;
  private double speedChangeScale = 50;
  PIDController leftPidController = new PIDController(Constants.kPDriveVel, 0, 0);


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
    boolean rightStickPressed = xboxController.getLeftStickButtonPressed();
    SmartDashboard.putNumber("Drive Speed", rightIndex + 1);
    
    //smooth drive
    //leftPidController.

    //x = (x - previousX) / speedChangeScale + previousX;
    //y = (y - previousY) / speedChangeScale + previousY;

    if (rightStickPressed) {
      rightIndex++;
      if(rightIndex > driveSpeedList.length - 1) {
        rightIndex = 0;
      }
      changeDriveSpeed = driveSpeedList[rightIndex];
    }

    //joystick deadzone
    if((x > deadzoneX || x < -deadzoneX) || (y > deadzoneY || y < -deadzoneY)){
      driveSubsystem.arcadeDrive(-y/changeDriveSpeed, x/changeDriveSpeed);
    } else {
      driveSubsystem.arcadeDrive(0, 0);
    }

    previousX = x;
    previousY = y;
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
