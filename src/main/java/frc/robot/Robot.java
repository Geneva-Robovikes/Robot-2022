// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.command.auto.autopaths.*;
//import frc.robot.command.drive.TeleopDriveCommand;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import com.ctre.phoenix.music.Orchestra;
//import frc.robot.subsystem.*;
//import frc.robot.DashHelper;
import edu.wpi.first.wpilibj.PowerDistribution;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj.XboxController;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private DriveSubsystem drive;
  public PowerDistribution pdp;

  public WPI_TalonFX motorRightFront;
  public WPI_TalonFX motorLeftFront;
  public WPI_TalonFX motorRightBack;
  public WPI_TalonFX motorLeftBack;
  public Orchestra orchestra;
  public XboxController xboxController;
  public int songselection;
  public String[] songList;
  public int btn;
  public int lastButton;
  public int selectedsong;

  public Robot() {
    xboxController = new XboxController(0);

    orchestra = new Orchestra();
    motorLeftFront = new WPI_TalonFX(0);
    motorRightFront = new WPI_TalonFX(2);
    motorLeftBack = new WPI_TalonFX(1);
    motorRightBack = new WPI_TalonFX(3);
    motorRightFront.setSafetyEnabled(false);
    motorRightBack.setSafetyEnabled(false);
    motorLeftFront.setSafetyEnabled(false);
    motorLeftBack.setSafetyEnabled(false);
    
    //10 song max for xbox controllers
    songList = new String[] {
      "Fanfare.chrp",
      "portal.chrp",
      "HyruleCastle.chrp",
      "Halo.chrp",
      "TurretSong.chrp",
      "Wiisports.chrp",
      "Zelda.chrp",
      "happyBirthday2.chrp",
      "Enemymelody.chrp"
    };
    songselection = 0;
    lastButton = 0;
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  void LoadMusicSelection(int button) {
    songselection = button;
    //System.out.println(songselection);
    if (songselection >= songList.length) {
      songselection = 0;
    }
    if (songselection < 0) {
      songselection = songList.length - 1;
    }
    orchestra.loadMusic(songList[songselection]);
    System.out.println(songList[songselection]);
  }

  int getButton() {
    for (int i = 1; i < 10; ++i) {
      if (xboxController.getRawButton(i)) {
        //System.out.println(xboxController.getRawButton(i));
        selectedsong = i-1;
      }
    }
    return(selectedsong);
  }

  @Override
  public void robotInit() {
    orchestra.loadMusic("Fanfare.chrp");
    orchestra.stop();

    orchestra.stop();
    orchestra.addInstrument(motorLeftBack);
    orchestra.addInstrument(motorLeftFront);
    orchestra.addInstrument(motorRightBack);
    orchestra.addInstrument(motorRightFront);
    orchestra.stop();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //robotContainer.driveStraight.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //robotContainer.teleopDrive.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    btn = getButton();

    if (lastButton != btn) {
      //System.out.println(btn);
      orchestra.stop();
      lastButton = btn;
      LoadMusicSelection(btn);
      orchestra.play();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
