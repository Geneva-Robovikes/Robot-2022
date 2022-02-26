package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private VictorSPX rollerMotor;

    public IntakeSubsystem () {
        rollerMotor = new VictorSPX(4);
    }

    public void setRollerMotor(double value) {
        rollerMotor.set(ControlMode.PercentOutput, value);
    }
}
