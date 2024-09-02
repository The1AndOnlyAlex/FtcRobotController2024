package commands;

import com.arcrobotics.ftclib.command.RunCommand;

import subsystems.FTCLibMecanumDriveSubsystem;

public class MecanumDriveStopCommand extends RunCommand {
    public MecanumDriveStopCommand(FTCLibMecanumDriveSubsystem driveSubsystem) {
        super(driveSubsystem::stop, driveSubsystem);
    }
}
