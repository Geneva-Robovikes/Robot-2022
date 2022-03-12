package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase {
    private WPI_VictorSPX beltMotor;
    private DigitalInput limitSwitch;

    public BeltSubsystem() {
        beltMotor = new WPI_VictorSPX(5);
        limitSwitch = new DigitalInput(0);
    }

    public void setBeltMotor(double value) {
        beltMotor.set(ControlMode.PercentOutput, value);
    }

    public boolean getSwitchState() {
        return limitSwitch.get();
    }
}
