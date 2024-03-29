// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import SPI

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX motorLeftFront = new WPI_TalonFX(0);
    private final WPI_TalonFX motorRightFront = new WPI_TalonFX(2);
    private final WPI_TalonFX motorLeftBack = new WPI_TalonFX(1);
    private final WPI_TalonFX motorRightBack = new WPI_TalonFX(3);
    public final ADXRS450_Gyro gyro = new ADXRS450_Gyro(); 

    private MotorControllerGroup driveLeft = new MotorControllerGroup(motorLeftFront, motorLeftBack);
    private MotorControllerGroup driveRight = new MotorControllerGroup(motorRightFront, motorRightBack);
    private DifferentialDrive differentialDrive;

    private XboxController controller;

    private double zeroAngle;

    private final DifferentialDriveOdometry odometry;

    public DriveSubsystem(){
      zeroAngle = 0;
      motorLeftFront.setSafetyEnabled(false);
      motorRightFront.setSafetyEnabled(false);
      motorLeftBack.setSafetyEnabled(false);
      motorRightBack.setSafetyEnabled(false);
      driveRight.setInverted(true);
      differentialDrive = new DifferentialDrive(driveLeft, driveRight);
      differentialDrive.setSafetyEnabled(false);
      gyro.reset();
      gyro.calibrate();
      odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    public void arcadeDrive(double speed, double rotation) {
      differentialDrive.arcadeDrive(speed, rotation);
    }
    public void tankDrive (double leftSpeed, double rightSpeed){
      differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void curvatureDrive (double speed, double rotation) {
      differentialDrive.curvatureDrive(speed, rotation, false);
    }

    public double getLeftX() {
      return controller.getLeftX();
    }
    
    public double getLeftY() {
      return controller.getLeftY();
    }

    public WPI_TalonFX getMotor(int i) {
      if(i == 0) {return motorLeftFront;}
      else {return motorRightBack;}
    }

    public double getEncoderMeters(WPI_TalonFX motor, int mult) {
      double wheelDiameter = 0.1524;
      double gearRatio = 10.71;
      double unitsPerRevolution = 2048;
      return mult * motor.getSelectedSensorPosition() / (gearRatio * unitsPerRevolution) * (Math.PI * wheelDiameter);
    }

    private double getEncoderVelocity(WPI_TalonFX motor, int mult) {
      double wheelDiameter = 0.1524;
      double gearRatio = 10.71;
      double unitsPerRevolution = 2048;
      return mult * motor.getSelectedSensorVelocity() / (gearRatio * unitsPerRevolution) * (Math.PI * wheelDiameter);
    }

    public double getLeftMotorVelocity() {
      double wheelDiameter = 0.1524;
      double gearRatio = 10.71;
      double unitsPerRevolution = 2048;
      return motorLeftFront.getSelectedSensorVelocity() / (gearRatio * unitsPerRevolution) * (Math.PI * wheelDiameter);
    }

    @Override
    public void periodic() {
      odometry.update(gyro.getRotation2d(), getEncoderMeters(motorLeftFront, 1), getEncoderMeters(motorRightFront, -1));
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(getEncoderVelocity(motorLeftFront, 1), getEncoderVelocity(motorRightFront, -1));
    }
    
    public void ResetOdometry(Pose2d pose) {
      motorRightFront.setSelectedSensorPosition(0);
      motorLeftFront.setSelectedSensorPosition(0);
      odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public double getHeading() {
      return gyro.getRotation2d().getDegrees();
    }

    public double getLeftEncoderMeters() {
      double wheelDiameter = 0.1524;
      double gearRatio = 10.71;
      double unitsPerRevolution = 2048;
      return motorLeftFront.getSelectedSensorPosition() / (gearRatio * unitsPerRevolution) * (Math.PI * wheelDiameter);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
      driveLeft.setVoltage(leftVolts);
      driveRight.setVoltage(rightVolts);
      differentialDrive.feed();
    }

    public double getGyro() {
      return gyro.getAngle();
    }

    public void setZeroAngle(double angle) {
      zeroAngle = angle;
    }

    public double getZeroAngle() {
      return zeroAngle;
    }

    public double getGyroRate() {
      return gyro.getRate();
    }
}
