package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;



public class BackwardPneumaticsCommand extends CommandBase {
  private final PneumaticsSubsystem pneumaticsSubsystem;

  public BackwardPneumaticsCommand(PneumaticsSubsystem subsystem) {
    pneumaticsSubsystem = subsystem;
    addRequirements(subsystem);


  }

  @Override
  public void initialize() {
    pneumaticsSubsystem.setSolenoid(kReverse);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    pneumaticsSubsystem.setSolenoid(kOff);
  }
}
