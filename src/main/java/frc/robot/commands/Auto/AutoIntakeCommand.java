// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.PneumaticsSubsystem;

/** An example command that uses an example subsystem. */
public class AutoIntakeCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  //private final PneumaticsSubsystem pneumaticsSubsystem;
  private Timer timer = new Timer();
  private double waitTime = 1.5;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoIntakeCommand(IntakeSubsystem intakeSubsystem/*, PneumaticsSubsystem pneumaticsSubsystem*/, double waitTime) {
    this.intakeSubsystem = intakeSubsystem;
    //this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.waitTime = waitTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem/*, pneumaticsSubsystem*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setInnerRollerMotor(-.6);
    //intakeSubsystem.setOuterRollerMotor(.8);
    //pneumaticsSubsystem.setSolenoid(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > waitTime) {
      intakeSubsystem.setInnerRollerMotor(0);
      //intakeSubsystem.setOuterRollerMotor(0);
      //pneumaticsSubsystem.setSolenoid(Value.kReverse);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setInnerRollerMotor(0);
    //intakeSubsystem.setOuterRollerMotor(0);
    //pneumaticsSubsystem.setSolenoid(Value.kReverse);
  }
}
