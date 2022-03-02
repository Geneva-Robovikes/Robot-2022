// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

<<<<<<< Updated upstream
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
=======
  //Add Commands Here!
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveStraightForTime driveStraightfortime = new DriveStraightForTime(driveSubsystem);
  public final TeleopDrive teleopDrive = new TeleopDrive(driveSubsystem, controller);
  public final DefaultCommand defaultCommand = new DefaultCommand(driveSubsystem);
  public final AutoTimer autoTimer = new AutoTimer();
  public final DriveStraight driveStraight = new DriveStraight(driveSubsystem, 1.15, .5);
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
>>>>>>> Stashed changes

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
<<<<<<< Updated upstream
  private void configureButtonBindings() {}
=======
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(controller, 3);
    intakeButton.toggleWhenPressed(intakeCommand);
  }
  
  public Command TrajectoryCommand() {
    driveSubsystem.gyro.reset();

    // ~~~~~~ Change string to filename of wanted path ~~~~~~ //
    String pathToRun = "one ball";

    Trajectory trajectory = new Trajectory();
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output/" + pathToRun + ".wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectory, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = 
        new RamseteCommand(
            trajectory,
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
    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.ResetOdometry(trajectory.getInitialPose());
>>>>>>> Stashed changes

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
