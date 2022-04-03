package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private WPI_TalonFX climbLeft;
    private WPI_TalonFX climbRight;
    
    public ClimbSubsystem() {
        climbLeft = new WPI_TalonFX(8);
        climbRight = new WPI_TalonFX(9);

        climbLeft.setNeutralMode(NeutralMode.Brake);
        climbRight.setNeutralMode(NeutralMode.Brake);
    }

    public void setClimbMotors(double speed) {
        climbLeft.set(ControlMode.PercentOutput, speed);
        climbRight.set(ControlMode.PercentOutput, -speed);
    }

    public double getRightClimbEncoder() {
        return climbRight.getSelectedSensorPosition();
    }
}
