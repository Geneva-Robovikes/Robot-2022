package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private WPI_VictorSPX climbLeft;
    private WPI_VictorSPX climbRight;

    //private TalonFX climbLeft;
    //private TalonFX climbRight;
    //make sure new talons are updated

    public ClimbSubsystem() {
        climbLeft = new WPI_VictorSPX(8);
        climbRight = new WPI_VictorSPX(9);

        //climbLeft = new TalonFX(8);
        //climbRight = new TalonFX(9);
    }

    public void setClimbMotors(double speed) {
        climbLeft.set(ControlMode.PercentOutput, speed);
        climbRight.set(ControlMode.PercentOutput, -speed);
    }
}
