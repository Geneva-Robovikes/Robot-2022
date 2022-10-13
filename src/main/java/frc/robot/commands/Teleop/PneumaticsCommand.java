package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


public class PneumaticsCommand extends CommandBase {
    private final PneumaticsSubsystem pneumaticsSubsystem;

    public PneumaticsCommand(PneumaticsSubsystem subsystem) {
        pneumaticsSubsystem = subsystem;
        addRequirements(subsystem);


    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.setSolenoid(Value.kForward);
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      pneumaticsSubsystem.setSolenoid(Value.kReverse);
    }

    //public void get

}
