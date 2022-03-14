// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ReplayTeleop extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Timer timer = new Timer();
  private int currentPoint = 0;
  private Points[] pointArray;

  class Points {
    private final double time;
    private final double leftVolts;
    private final double rightVolts;

    public Points(double time, double leftVolts, double rightVolts) {
      this.time = time;
      this.leftVolts = leftVolts;
      this.rightVolts = rightVolts;
    }
  }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReplayTeleop(DriveSubsystem subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String filename = "";
    try {
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve("Recordings/" + filename + ".json");
      Reader reader = Files.newBufferedReader(filePath, StandardCharsets.UTF_8);
      Gson gson = new GsonBuilder().create();
      pointArray = gson.fromJson(reader, Points[].class);
    } catch (IOException exception) {
      DriverStation.reportError("Oopsie!", exception.getStackTrace());
    }
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pointArray[currentPoint].time > timer.get()) {
      driveSubsystem.tankDriveVolts(pointArray[currentPoint].leftVolts, pointArray[currentPoint].rightVolts);
      currentPoint++;
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
    if(pointArray.length-1 == currentPoint) { return true; }
    return false;
  }
}
