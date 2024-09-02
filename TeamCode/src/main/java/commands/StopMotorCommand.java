package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.BasicSubsystem;

public class StopMotorCommand extends CommandBase {
    private BasicSubsystem subsystem;

    public StopMotorCommand(BasicSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
