package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private VictorSPX rollerMotor;
    private VictorSPX beltMotor;

    public IntakeSubsystem () {
        rollerMotor = new VictorSPX(4);
        beltMotor = new VictorSPX(5);
    }

    public void setRollerMotor(double value) {
        rollerMotor.set(ControlMode.PercentOutput, value);
    }

    public void setBeltMotor(double value) {
        beltMotor.set(ControlMode.PercentOutput, value);
    }
}
