package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchSubsystem extends SubsystemBase {
    private VictorSPX leftMotor;
    private VictorSPX rightMotor;

    public LaunchSubsystem () {
        leftMotor = new VictorSPX(6);
        rightMotor = new VictorSPX(7);
    }

    public void setLaunchMotors(double value) {
        leftMotor.set(ControlMode.PercentOutput, -value);
        rightMotor.set(ControlMode.PercentOutput, -value);
    }
}
