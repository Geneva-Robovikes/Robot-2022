package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {
    private WPI_VictorSPX leftMotor;
    private WPI_VictorSPX rightMotor;

    public LaunchSubsystem () {
        leftMotor = new WPI_VictorSPX(6);
        rightMotor = new WPI_VictorSPX(7);
    }

    public void setLaunchMotors(double value) {
        leftMotor.set(ControlMode.PercentOutput, -value);
        rightMotor.set(ControlMode.PercentOutput, -value);
    }
}
