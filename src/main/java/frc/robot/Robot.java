// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
//import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.util.sendable.Sendable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.command.auto.FollowPathCommand;
//import frc.robot.command.auto.autopaths.*;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.command.intake.DeployIntake;
//import frc.robot.command.intake.RollerOffCommand;
//import frc.robot.command.intake.RollerOnCommand;
//import frc.robot.path.PathDataModel;
//import frc.robot.subsystem.*;
//import frc.robot.DashHelper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Robot extends TimedRobot {
  private DriveSubsystem drive;
  //private IntakeSubsystem intake;
  public PowerDistribution pdp;
  //private double beginningPosition = 0;

  //private double currentPosition = Math.abs(drive.motorLeftBack.getSelectedSensorPosition() - beginningPosition);

  //public DashHelper dash;

  public WPI_TalonFX motorRightFront;
  public WPI_TalonFX motorLeftFront;
  public WPI_TalonFX motorRightBack;
  public WPI_TalonFX motorLeftBack;

  public ADIS16448_IMU gyro;

  public XboxController xboxController;

  private final Field2d m_field = new Field2d();


  public Robot() {
    xboxController = new XboxController(0);
    // TODO: refactor port numbers into variables
    pdp = new PowerDistribution();
    pdp.clearStickyFaults();
    SmartDashboard.putData("Field", m_field);

    
    //DashHelper.getInstance().setUpPDPWidget(pdp);
    //DashHelper.getInstance().setUpGyroWidget(gyro);
    //DashHelper.getInstance().setEncoder(currentPosition);
    //DashHelper.getInstance().

    System.out.println("Robot.Robot(): initializing motorRightFront");
    motorRightFront = new WPI_TalonFX(0);

    System.out.println("Robot.Robot(): initializing motorLeftFront");
    motorRightBack = new WPI_TalonFX(1);

    System.out.println("Robot.Robot(): initializing motorRightBack");
    motorLeftFront = new WPI_TalonFX(2);

    System.out.println("Robot.Robot(): initializing motorLeftBack");
    motorLeftBack = new WPI_TalonFX(3);

    System.out.println("Robot.Robot(): initialized all motors");

    gyro = new ADIS16448_IMU();

    drive = new DriveSubsystem(motorRightFront, motorLeftFront, motorRightBack, motorLeftBack, gyro);
    //intake = new IntakeSubsystem();
    drive.m_right.setInverted(true);
  } 

  @Override
  public void robotInit() {

  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
  /*
    String filePath = new File("").getAbsolutePath();
    filePath = filePath.concat("\\src\\main\\test\\frc\\robot\\path\\CircuitPathFixture.wpilib.json");
    */
    /*String filePath = "/usr/paths/CircuitPathFixture.wpilib.json";
    PathDataModel pathDataModel = new PathDataModel(PathDataModel.readFromInputStream(filePath));
    */
    //CommandScheduler.getInstance().schedule(new BarrelRacingPathCommandGroup(drive));
  }


  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(new TeleopDriveCommand(drive, xboxController));
    /*joystick.getButton(5).whenPressed(new RollerOnCommand(intake));
    joystick.getButton(6).whenPressed(new RollerOffCommand(intake));
    joystick.getButton(11).whenPressed(new DeployIntake(intake));*/
  }


  @Override
  public void teleopPeriodic() {
    //m_field.setRobotPose(m_odometry.getPoseMeters());
  }


  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}


  @Override
  public void testInit() {}


  @Override
  public void testPeriodic() {}
}