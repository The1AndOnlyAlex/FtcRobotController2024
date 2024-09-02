package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import subsystems.IntakeSubsystem;

/**
 * A simple command that grabs a stone with the {@link IntakeSubsystem}.  Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * com.arcrobotics.ftclib.command.InstantCommand}.
 */
public class IntakeGrabCommand extends CommandBase {

    // The subsystem the command runs on
    private final IntakeSubsystem m_gripperSubsystem;

    Telemetry telemetry;
    int a = 100;

    public IntakeGrabCommand(IntakeSubsystem subsystem, Telemetry telemetry) {
        m_gripperSubsystem = subsystem;
        this.telemetry = telemetry;
        addRequirements(m_gripperSubsystem);
    }

    @Override
    public void initialize() {
        telemetry.addData("Grab Command: ", a);
        m_gripperSubsystem.grab();
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
