package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import subsystems.MecanumDriveSubsystem;
import util.OldDriverFilter2;
import util.filters.DeadbandFilter;
import util.filters.FilterSeries;
import util.filters.ScaleFilter;

public class DriverJoystickCommand extends CommandBase {

    private final MecanumDriveSubsystem m_drive;

    DoubleSupplier xSpdSupplier;
    DoubleSupplier ySpdSupplier;
    DoubleSupplier rotationSpdSupplier;
    BooleanSupplier fieldOrientedSupplier;
    BooleanSupplier robotOrientedSupplier;
    BooleanSupplier resetCurrentHeading2Zero;
    BooleanSupplier towLeftSupplier;
    BooleanSupplier towRightSupplier;
    DoubleSupplier precisionLeftSupplier;
    DoubleSupplier precisionRightSupplier;
    BooleanSupplier turnToForwardSupplier;
    BooleanSupplier turnToBackwardSupplier;
    BooleanSupplier turnToLeftSupplier;
    BooleanSupplier turnToRightSupplier;

    DoubleSupplier currentHeadingSupplier;

    public DriverJoystickCommand(
            DoubleSupplier xSpdFunction,
            DoubleSupplier ySpdFunction,
            DoubleSupplier rotationSpdFunction,
            BooleanSupplier fieldOrientedFunction,
            BooleanSupplier robotOrientedFunction,
            BooleanSupplier resetCurrentHeading2Zero,
            BooleanSupplier towLeftSupplier,
            BooleanSupplier towRightSupplier,
            DoubleSupplier precisionLeftSupplier,
            DoubleSupplier precisionRightSupplier,
            BooleanSupplier turnToForwardSupplier,
            BooleanSupplier turnToBackwardSupplier,
            BooleanSupplier turnToLeftSupplier,
            BooleanSupplier turnToRightSupplier,
            DoubleSupplier currentHeadingPI2NPI,
            MecanumDriveSubsystem drive)
    {
        this.xSpdSupplier = xSpdFunction;
        this.ySpdSupplier = ySpdFunction;
        this.rotationSpdSupplier = rotationSpdFunction;
        this.fieldOrientedSupplier = fieldOrientedFunction;
        this.robotOrientedSupplier = robotOrientedFunction;
        this.resetCurrentHeading2Zero = resetCurrentHeading2Zero;
        this.towLeftSupplier = towLeftSupplier;
        this.towRightSupplier = towRightSupplier;
        this.precisionLeftSupplier = precisionLeftSupplier;
        this.precisionRightSupplier = precisionRightSupplier;
        this.turnToForwardSupplier = turnToForwardSupplier;
        this.turnToBackwardSupplier = turnToBackwardSupplier;
        this.turnToLeftSupplier = turnToLeftSupplier;
        this.turnToRightSupplier = turnToRightSupplier;
        this.currentHeadingSupplier = currentHeadingPI2NPI;
        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        //m_drive.resetEncoders();
    }

    boolean doAutoHeading = false;
    double targetAutoHeading = 0;
    @Override
    public void execute() {
        if(resetCurrentHeading2Zero.getAsBoolean())
        {
            m_drive.resetHeading2Zero();
        }
        // Retrieve real-time inputs from joystick and button states
        double xSpeed = xSpdSupplier.getAsDouble();
        double ySpeed = ySpdSupplier.getAsDouble();
        double rotationSpeed = rotationSpdSupplier.getAsDouble();

        OldDriverFilter2 xFilter = new OldDriverFilter2(
                0.05,//ControllerConstants.kDeadband,
                0.05,//kMinimumMotorOutput,
                5,//kTeleDriveMaxSpeedMetersPerSecond,
                0.11765,//kDriveAlpha,
                5,//kTeleMaxAcceleration,
                -5);//kTeleMaxDeceleration);
        OldDriverFilter2 yFilter = new OldDriverFilter2(
                0.05,//ControllerConstants.kDeadband,
                0.05,//kMinimumMotorOutput,
                5,//kTeleDriveMaxSpeedMetersPerSecond,
                0.11765,//kDriveAlpha,
                5,//kTeleMaxAcceleration,
                -5);//kTeleMaxDeceleration);
        FilterSeries turningFilter = new FilterSeries(
                new DeadbandFilter(0.1),//ControllerConstants.kRotationDeadband),
                new ScaleFilter(12.566)//kTeleDriveMaxAngularSpeedRadiansPerSecond)
        );

        double filteredXSpeed = xFilter.calculate(xSpeed);
        double filteredYSpeed = yFilter.calculate(ySpeed);
        double filteredTurningSpeed;

        // Retrieve current heading and control mode states
        double currentHeading = currentHeadingSupplier.getAsDouble();

        if( fieldOrientedSupplier.getAsBoolean()) {
            m_drive.setFieledRelative(true);
        }
        if( robotOrientedSupplier.getAsBoolean()) {
            m_drive.setFieledRelative(false);
        }

        boolean towMode = towLeftSupplier.getAsBoolean() || towRightSupplier.getAsBoolean();

        double precisionMode = Math.max(
                precisionLeftSupplier.getAsDouble(), precisionRightSupplier.getAsDouble() );
        if (precisionMode > 0.001) {
            precisionMode = Math.abs(1-precisionMode);
            if(precisionMode < 0.3) precisionMode = 0.3;
            m_drive.setMaxOutput(precisionMode);
        }

        // Implement tow mode adjustments
        if (towMode) {
            xSpeed *= 0.001;  // Reduce speed for towing
            ySpeed *= 0.001;
            rotationSpeed *= 0.001;
        }

        // Handling combinations of directional inputs
        boolean forward = turnToForwardSupplier.getAsBoolean();
        boolean backward = turnToBackwardSupplier.getAsBoolean();
        boolean left = turnToLeftSupplier.getAsBoolean();
        boolean right = turnToRightSupplier.getAsBoolean();

        // Initialize variables to track state and compute the target heading
        double targetHeading = 0;
        int directionCount = 0;

        // Calculate target heading based on button presses
        if (forward) {
            directionCount++;
            targetHeading += 0; // Forward is 0 degrees
        }
        if (backward) {
            directionCount++;
            targetHeading += 180; // Backward is 180 degrees
        }
        if (left) {
            directionCount++;
            targetHeading += 90; // Left is 90 degrees
        }
        if (right) {
            directionCount++;
            targetHeading -= 90; // Right is -90 degrees
        }

        if (directionCount > 0) {
            targetAutoHeading = targetHeading / directionCount; // Average if multiple buttons are pressed
        }

        // Normalize the target heading to be within -180 to 180 degrees
        targetAutoHeading = ((targetAutoHeading + 180) % 360) - 180;

        // Update auto-heading flag
        doAutoHeading = (directionCount > 0);

        // Apply automatic heading adjustments if required
        if (doAutoHeading) {
            m_drive.adjustToHeading(targetAutoHeading, currentHeadingSupplier.getAsDouble());
            doAutoHeading = false; // Reset the flag after adjustment begins
        } else {
            // Continue with the regular driving command
            m_drive.drive(xSpeed, ySpeed, rotationSpeed, false, currentHeadingSupplier.getAsDouble());
        }
    }
}
