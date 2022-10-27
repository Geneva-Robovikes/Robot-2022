package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private WPI_VictorSPX rollerMotor;

    public IntakeSubsystem () {
        rollerMotor = new WPI_VictorSPX(4);
    }

    public void setRollerMotor(double value) {
        rollerMotor.set(ControlMode.PercentOutput, value);
    }
}
