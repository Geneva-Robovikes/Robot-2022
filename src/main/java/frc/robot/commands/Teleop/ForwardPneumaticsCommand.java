package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;



public class ForwardPneumaticsCommand extends CommandBase {
  private final PneumaticsSubsystem pneumaticsSubsystem;

  public ForwardPneumaticsCommand(PneumaticsSubsystem subsystem) {
    pneumaticsSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    pneumaticsSubsystem.setSolenoid(kForward);
    System.out.println("print");
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
