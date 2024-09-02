package commands;

import com.arcrobotics.ftclib.command.RunCommand;

import subsystems.TankDriveSubsystem;

public class DriveCommand extends RunCommand {
    public DriveCommand(TankDriveSubsystem driveSubsystem) {
        super(driveSubsystem::drive, driveSubsystem);
    }
}
