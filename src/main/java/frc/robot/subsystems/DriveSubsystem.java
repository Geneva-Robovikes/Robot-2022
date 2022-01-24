// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
    private final PWMTalonFX motorLeftFront = new PWMTalonFX(0);
    private final PWMTalonFX motorRightFront = new PWMTalonFX(1);
    private final PWMTalonFX motorLeftBack = new PWMTalonFX(2);
    private final PWMTalonFX motorRightBack = new PWMTalonFX(3);

    private MotorControllerGroup driveLeft;
    private MotorControllerGroup driveRight;
    private DifferentialDrive differentialDrive;

    public DriveSubsystem(){
      driveLeft = new MotorControllerGroup(motorLeftFront, motorLeftBack);
      driveRight = new MotorControllerGroup(motorRightFront, motorRightBack);
      differentialDrive = new DifferentialDrive(driveLeft, driveRight);
    }

    public void setMotors(float leftSpeed, float rightSpeed){
      differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }
}
