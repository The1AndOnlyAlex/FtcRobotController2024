package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.IntakeSubsystem;

/**
 * A simple command that releases a stone with the {@link IntakeReleaseCommand}.  Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * com.arcrobotics.ftclib.command.InstantCommand}.
 */
public class IntakeReleaseCommand extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem m_gripperSubsystem;

    public IntakeReleaseCommand(IntakeSubsystem subsystem) {
        m_gripperSubsystem = subsystem;
        addRequirements(m_gripperSubsystem);
    }

    @Override
    public void initialize() {
        m_gripperSubsystem.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
