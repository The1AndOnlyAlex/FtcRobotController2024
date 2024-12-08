package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import subsystems.MecanumDriveSubsystem;

public class DriverJoystickCommand extends CommandBase {

    private final MecanumDriveSubsystem m_drive;

    double xSpdFunction;
    double ySpdFunction;
    double rotationSpdFunction;
    boolean fieldOrientedFunction;
    boolean towSupplier;
    double precisionSupplier;
    boolean turnToForwardSupplier;
    boolean turnToBackwardSupplier;
    boolean turnToLeftSupplier;
    boolean turnToRightSupplier;

    double currentHeadingPI2NPI;

    public DriverJoystickCommand(
            DoubleSupplier xSpdFunction,
            DoubleSupplier ySpdFunction,
            DoubleSupplier rotationSpdFunction,
            BooleanSupplier fieldOrientedFunction,
            BooleanSupplier towSupplier,
            DoubleSupplier precisionSupplier,
            BooleanSupplier turnToForwardSupplier,
            BooleanSupplier turnToBackwardSupplier,
            BooleanSupplier turnToLeftSupplier,
            BooleanSupplier turnToRightSupplier,
            DoubleSupplier currentHeadingPI2NPI,
            MecanumDriveSubsystem drive)
    {
        this.xSpdFunction = xSpdFunction.getAsDouble();
        this.ySpdFunction = ySpdFunction.getAsDouble();
        this.rotationSpdFunction = rotationSpdFunction.getAsDouble();
        this.fieldOrientedFunction = fieldOrientedFunction.getAsBoolean();
        this.towSupplier = towSupplier.getAsBoolean();
        this.precisionSupplier = precisionSupplier.getAsDouble();
        this.turnToForwardSupplier = turnToForwardSupplier.getAsBoolean();
        this.turnToBackwardSupplier = turnToBackwardSupplier.getAsBoolean();
        this.turnToLeftSupplier = turnToLeftSupplier.getAsBoolean();
        this.turnToRightSupplier = turnToRightSupplier.getAsBoolean();
        this.currentHeadingPI2NPI = currentHeadingPI2NPI.getAsDouble();
        m_drive = drive;
    }

    @Override
    public void initialize() {
        //m_drive.resetEncoders();
    }

    boolean doAutoHeading = false;
    double targetAutoHeading = 0;
    @Override
    public void execute()
    {
        if(turnToBackwardSupplier || turnToForwardSupplier || turnToRightSupplier || turnToLeftSupplier)
        {
            doAutoHeading = true
        }
        m_drive.drive(xSpdFunction, ySpdFunction, rotationSpdFunction,
                currentHeadingPI2NPI, fieldOrientedFunction);
    }

//    @Override
//    public void end(boolean interrupted) {
//        m_drive.stop();
//    }
//
//
//    @Override
//    public boolean isFinished() {
//        return Math.abs(m_drive.getCurrentAngleDegree()) >= m_angle;
//    }

}
