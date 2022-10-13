// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
    private PWMTalonFX motorLeftFront;
    private PWMTalonFX motorRightFront;
    private PWMTalonFX motorLeftBack;
    private PWMTalonFX motorRightBack;
    private ADIS16448_IMU gyro;

    public MotorControllerGroup driveLeft;
    public MotorControllerGroup driveRight;
    public DifferentialDrive differentialDrive;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem(
    PWMTalonFX motorLeftFront, PWMTalonFX motorRightFront,
    PWMTalonFX motorLeftBack, PWMTalonFX motorRightBack,
    ADIS16448_IMU gyro) {
      this.motorLeftFront = motorLeftFront;
      this.motorRightFront = motorRightFront;
      this.motorLeftBack = motorLeftBack;
      this.motorRightBack = motorRightBack;
      this.gyro = gyro;

      driveLeft = new MotorControllerGroup(motorLeftFront, motorLeftBack);
      driveRight = new MotorControllerGroup(motorRightFront, motorRightBack);
      differentialDrive = new DifferentialDrive(driveLeft, driveRight);
    }
  
  public void tankDrive (double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
