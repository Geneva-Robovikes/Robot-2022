// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto.AutoBeltCommand;
import frc.robot.commands.Auto.AutoContinuousBeltCommand;
import frc.robot.commands.Auto.AutoIntakeCommand;
import frc.robot.commands.Auto.AutoLaunchCommand;
import frc.robot.commands.Auto.AutoTimer;
import frc.robot.commands.Auto.DriveStraightForTime;
import frc.robot.commands.Auto.DriveStraightPIDCommand;
import frc.robot.commands.Auto.TurnPIDCommand;
import frc.robot.commands.Teleop.BeltCommand;
import frc.robot.commands.Teleop.DefaultCommand;
import frc.robot.commands.Teleop.IntakeCommand;
import frc.robot.commands.Teleop.LaunchCommand;
import frc.robot.commands.Teleop.TeleopDrive;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
//import frc.robot.subsystems.ClimbSubsystem;
//import frc.robot.commands.Teleop.ClimbCommandDown;
//import frc.robot.commands.Teleop.ClimbCommandUp;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController controller = new XboxController(0);

  //Add Subsystems Here!
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final LaunchSubsystem launchSubsystem = new LaunchSubsystem();
  public final BeltSubsystem beltSubsystem = new BeltSubsystem();
  //public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  //Add Teleop Commands Here!
  public final TeleopDrive teleopDrive = new TeleopDrive(driveSubsystem, controller);
  public final DefaultCommand defaultCommand = new DefaultCommand(driveSubsystem);
  public final BeltCommand beltCommand = new BeltCommand(beltSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  public final LaunchCommand launchCommand = new LaunchCommand(launchSubsystem, controller);
  //public final ClimbCommandUp climbCommandUp = new ClimbCommandUp(climbSubsystem);
  //public final ClimbCommandDown climbCommandDown = new ClimbCommandDown(climbSubsystem);
  
  //add Auto Commands Here!
  public final DriveStraightForTime driveStraightfortime = new DriveStraightForTime(driveSubsystem);
  public final DriveStraightPIDCommand driveStraightPIDCommand = new DriveStraightPIDCommand(driveSubsystem, 1.15);
  public final AutoTimer autoTimer = new AutoTimer();
  public final TurnPIDCommand turnPIDCommand = new TurnPIDCommand(driveSubsystem, 180);
  public final AutoIntakeCommand autoIntakeCommand = new AutoIntakeCommand(intakeSubsystem);
  public final AutoBeltCommand autoBeltCommand = new AutoBeltCommand(beltSubsystem, 0.8);
  public final AutoLaunchCommand autoLaunchCommand = new AutoLaunchCommand(launchSubsystem);
  public final AutoContinuousBeltCommand autoContinuousBeltCommand = new AutoContinuousBeltCommand(beltSubsystem);

  //private Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, defaultCommand);
    //autoCommand = new ParallelRaceGroup(TrajectoryCommand(), autoTimer);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(controller, 1);
    JoystickButton launchButton = new JoystickButton(controller, 4);
    JoystickButton beltButton = new JoystickButton(controller, 3);
    //JoystickButton climbDownButton = new JoystickButton(controller, 5);
    //JoystickButton climbUpButton = new JoystickButton(controller, 6);

    intakeButton.toggleWhenPressed(intakeCommand);
    launchButton.toggleWhenPressed(launchCommand);
    beltButton.toggleWhenPressed(beltCommand);
    //climbDownButton.toggleWhenPressed(climbCommandDown);
    //climbUpButton.toggleWhenPressed(climbCommandUp);
  }
/*
  public Command AutoCommand() {
    return autoCommand;
  }
*/
  public Command TrajectoryCommand() {
    driveSubsystem.gyro.reset();

    // ~~~~~~ Change this string to the path you want to run ~~~~~~//
    String pathToRun = "F5";
    //String secondPath = "one ball part2";
    
    Trajectory trajectory1 = new Trajectory();
    //Trajectory trajectory2 = new Trajectory();
    try {
      Path path1 = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output/" + pathToRun + ".wpilib.json");
      //Path path2 = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output/" + secondPath + ".wpilib.json");

      trajectory1 = TrajectoryUtil.fromPathweaverJson(path1);
      //trajectory2 = TrajectoryUtil.fromPathweaverJson(path2);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectory1, ex.getStackTrace());
    }

    RamseteCommand ramseteCommandPart1 = 
        new RamseteCommand(
            trajectory1,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveSubsystem::tankDriveVolts,
            driveSubsystem);

            /*RamseteCommand ramseteCommandPart2 = 
            new RamseteCommand(
                trajectory2,
                driveSubsystem::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                driveSubsystem::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts,
                driveSubsystem);*/
    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.ResetOdometry(trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return new ParallelRaceGroup(new AutoIntakeCommand(intakeSubsystem), new AutoBeltCommand(beltSubsystem, 8.0), ramseteCommandPart1.andThen(() -> driveSubsystem.tankDriveVolts(0, 0))).andThen(new ParallelCommandGroup(new AutoLaunchCommand(launchSubsystem), new AutoContinuousBeltCommand(beltSubsystem)));
    //return new ParallelCommandGroup(new ParallelRaceGroup(autoIntakeCommand, ramseteCommandPart1.andThen(new ParallelCommandGroup(autoBeltCommand, autoIntakeCommand, ramseteCommandPart2)).andThen(() -> driveSubsystem.tankDriveVolts(0, 0))),  autoTimer);
  }
}