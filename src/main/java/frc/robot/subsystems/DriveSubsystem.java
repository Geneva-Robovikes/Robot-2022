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
    //public final ADIS16448_IMU gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);


    private MotorControllerGroup driveLeft = new MotorControllerGroup(motorLeftFront, motorLeftBack);
    private MotorControllerGroup driveRight = new MotorControllerGroup(motorRightFront, motorRightBack);
    private DifferentialDrive differentialDrive;

    private XboxController controller;

    private final DifferentialDriveOdometry odometry;

    public DriveSubsystem(){
      driveRight.setInverted(true);
      motorLeftFront.setSafetyEnabled(false);
      motorRightFront.setSafetyEnabled(false);
      motorLeftBack.setSafetyEnabled(false);
      motorRightBack.setSafetyEnabled(false);
      differentialDrive = new DifferentialDrive(driveLeft, driveRight);
      differentialDrive.setSafetyEnabled(false);
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

    public double getEncoderMeters(WPI_TalonFX motor) {
      double wheelDiameter = 0.1524;
      double gearRatio = 10.71;
      double unitsPerRevolution = 2048;
      return motor.getSelectedSensorPosition() / (gearRatio * unitsPerRevolution / (Math.PI * wheelDiameter));
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getEncoderMeters(motorLeftFront), getEncoderMeters(motorRightFront));
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      return new DifferentialDriveWheelSpeeds(motorLeftFront.getSelectedSensorVelocity(), motorRightFront.getSelectedSensorVelocity());
    }

    public void ResetOdometry(Pose2d pose){
      motorRightFront.setSelectedSensorPosition(0);
      motorLeftFront.setSelectedSensorPosition(0);
      odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public double getHeading() {
      return gyro.getRotation2d().getDegrees();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
      driveLeft.setVoltage(leftVolts);
      driveRight.setVoltage(rightVolts);
      differentialDrive.feed();
    }

    public double getGyro() {
      return gyro.getAngle();
    }
}
