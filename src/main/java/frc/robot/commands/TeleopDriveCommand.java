package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
    private DriveSubsystem drive;
    //private Joystick joystick;
    private XboxController xboxController;

    public TeleopDriveCommand(DriveSubsystem drive, XboxController xboxController) {
        this.drive = drive;
        this.xboxController = xboxController;
        addRequirements(drive);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        double x, y;
        x = xboxController.getLeftX();
        y = xboxController.getLeftY();
        //z = xboxController.ge();

        drive.differentialDrive.arcadeDrive(x/2, y/2);
    }
}