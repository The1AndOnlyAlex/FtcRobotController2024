package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.BasicSubsystem;

public class RunMotorCommand extends CommandBase {
    private BasicSubsystem subsystem;

    public RunMotorCommand(BasicSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.runMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
