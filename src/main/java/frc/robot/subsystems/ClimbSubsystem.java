package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private WPI_VictorSPX climbLeft;
    private WPI_VictorSPX climbRight;

    public ClimbSubsystem() {
        climbLeft = new WPI_VictorSPX(8);
        climbRight = new WPI_VictorSPX(9);
    }
}
