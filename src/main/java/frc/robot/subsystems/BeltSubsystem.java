package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase {
    private VictorSPX beltMotor;

    public BeltSubsystem() {
        beltMotor = new VictorSPX(5);
    }

    public void setBeltMotor(double value) {
        beltMotor.set(ControlMode.PercentOutput, value);
    }
}
