// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonFX motorLeftFront = new WPI_TalonFX(0);
    private final WPI_TalonFX motorRightFront = new WPI_TalonFX(1);
    private final WPI_TalonFX motorLeftBack = new WPI_TalonFX(2);
    private final WPI_TalonFX motorRightBack = new WPI_TalonFX(3);

    private MotorControllerGroup driveLeft = new MotorControllerGroup(motorLeftFront, motorLeftBack);
    private MotorControllerGroup driveRight = new MotorControllerGroup(motorRightFront, motorRightBack);
    private DifferentialDrive differentialDrive;

    public DriveSubsystem(){
      driveRight.setInverted(true);
      differentialDrive = new DifferentialDrive(driveLeft, driveRight);
    }

    public void setMotors(double leftSpeed, double rightSpeed){
      differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }
}
