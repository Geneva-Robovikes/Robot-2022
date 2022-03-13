// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;
import frc.robot.subsystems.DriveSubsystem;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import com.google.gson.FieldNamingPolicy;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDriveRecord extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Gson gson = new GsonBuilder().setPrettyPrinting().setFieldNamingPolicy(FieldNamingPolicy.UPPER_CAMEL_CASE).create();
  private OutputStreamWriter writer;
  private double timeInterval = 0.01;
  private double prevTime;
  private Timer recordTimer = new Timer();
  private XboxController xboxController;
  private double halfSpeed = 2;
  private double threeQuarters = 1.33333;
  private double[] driveSpeedList = new double[2];
  private int rightIndex = 0;
  //private double inBetween = (1.6);
  //private double controllerScaleR = (1/.53);
  private double deadzoneX = 0.5;
  private double deadzoneY = 0.5;
  private double changeDriveSpeed = 2;

  class Item {
    
    private final String name;
    private final double quantity;

    public Item(String name, double quantity) {
        this.name = name;
        this.quantity = quantity;
    }
}

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDriveRecord(DriveSubsystem subsystem, XboxController controller) {
    xboxController = controller;
    driveSubsystem = subsystem;
    driveSpeedList[0] = halfSpeed;
    driveSpeedList[1] = threeQuarters;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recordTimer.start();

    try {
      // ~~~~~~ Filename of file to record to ~~~~~~ //
      String filename = "test";
      FileOutputStream fileOutput = new FileOutputStream("Recordings/" + filename + ".json");
      writer = new OutputStreamWriter(fileOutput, StandardCharsets.UTF_8);
    } catch (IOException ex) {
      DriverStation.reportError("Oh No!", ex.getStackTrace());
    }
  }

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

    if(recordTimer.get() > prevTime + timeInterval){
      prevTime = recordTimer.get();
      List<Item> data = new ArrayList<Item>();
      data.add(new Item("Time", prevTime));
      data.add(new Item("LeftVolts", driveSubsystem.getLeftVolts()));
      data.add(new Item("RightVolts", driveSubsystem.getRightVolts()));
      gson.toJson(data, writer);
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
