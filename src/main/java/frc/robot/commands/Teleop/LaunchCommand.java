// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;

/** An example command that uses an example subsystem. */
public class LaunchCommand extends CommandBase {
  private final LaunchSubsystem launchSubsystem;
  private XboxController xboxController;
  private int leftIndex = 0;
  private double speed;
  private double[] launchSpeedList = new double[3];
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LaunchCommand(LaunchSubsystem subsystem, XboxController controller) {
    xboxController = controller;
    launchSubsystem = subsystem;

    // ~~~~~ Launch Speeds ~~~~~ //
    launchSpeedList[0] = .25;
    //launchSpeedList[1] = .35;
    launchSpeedList[1] = .35;
    launchSpeedList[2] = .45;
    //launchSpeedList[1] = 0.4;
    //launchSpeedList[2] = 0.6;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean leftStickPressed = xboxController.getRightStickButtonPressed();
    SmartDashboard.putNumber("Launch Speed", leftIndex + 1);
    
    //it's actually right stick
    if (leftStickPressed) {
      leftIndex++;
      System.out.println("pressed");
      if(leftIndex > launchSpeedList.length - 1) {
        leftIndex = 0;
      }
    }
    speed = launchSpeedList[leftIndex];
    /*if (xboxController.getPOV() == 1) {
      speed += .05;
    }
    if (xboxController.getPOV() == 180) {
      speed -= .05;
    }*/
    //launchSubsystem.setLaunchMotors(speed);
    launchSubsystem.setLaunchMotors(launchSpeedList[leftIndex]);
    System.out.println(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    launchSubsystem.setLaunchMotors(0);
  }
}
