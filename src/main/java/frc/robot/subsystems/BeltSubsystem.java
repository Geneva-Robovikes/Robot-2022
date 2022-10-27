package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase {
    private WPI_VictorSPX beltMotor;
    private DigitalInput limitSwitch1;
    private DigitalInput limitSwitch2;

    public BeltSubsystem() {
        beltMotor = new WPI_VictorSPX(5);
        limitSwitch1 = new DigitalInput(0);
        limitSwitch2 = new DigitalInput(1);
    }

    public void setBeltMotor(double value) {
        beltMotor.set(ControlMode.PercentOutput, value);
    }

    public boolean getSwitch1State() {
        return limitSwitch1.get();
    }

    public boolean getSwitch2State() {
        return limitSwitch2.get();
    }
}
