package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private WPI_VictorSPX innerRollerMotor;
    //private CANSparkMax outerRollerMotor;

    public IntakeSubsystem () {
        //outerRollerMotor = new CANSparkMax(10, MotorType.kBrushless);
        innerRollerMotor = new WPI_VictorSPX(4);
    }

    public void setInnerRollerMotor(double value) {
        innerRollerMotor.set(ControlMode.PercentOutput, value);
    }

    /*public void setOuterRollerMotor(double value) {
        outerRollerMotor.set(value);
    }*/
}
