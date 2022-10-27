package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class PneumaticsSubsystem extends SubsystemBase{
    Compressor pcmCompressor;
    Compressor phCompressor;
    boolean enabled;
    boolean pressureSwitch;
    double current;
    DoubleSolenoid exampleDoublePCM;
    
    public PneumaticsSubsystem() {
        pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        enabled = pcmCompressor.enabled();
        
        pressureSwitch = pcmCompressor.getPressureSwitchValue();
        current = pcmCompressor.getCurrent();

        exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        exampleDoublePCM.set(kForward);
    }

    public boolean getEnabled() {
        return pcmCompressor.enabled();
    }

    public boolean getPressureSwitch() {
        return pcmCompressor.getPressureSwitchValue();
    }

    public double getCurrent() {
        return pcmCompressor.getCurrent();
    }

    public void setSolenoid(Value value) {
        exampleDoublePCM.set(value);
        System.out.println("Solenoid set to " + value);
    }
}


