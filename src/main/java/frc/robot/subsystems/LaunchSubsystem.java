package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {
    //private CANSparkMax leftMotor;
    //private CANSparkMax rightMotor;
    //private MotorType kMotorType;

    

    public LaunchSubsystem () {
        //kMotorType = MotorType.kBrushless;
        //leftMotor = new CANSparkMax(6, kMotorType);
        //rightMotor = new CANSparkMax(7, kMotorType);

    }

    public void setLaunchMotors(double value) {
        //leftMotor.set(ControlMode.PercentOutput, -value);
        //rightMotor.set(ControlMode.PercentOutput, -value);
        //leftMotor.set(value);
        //rightMotor.set(-value);
    }
}
