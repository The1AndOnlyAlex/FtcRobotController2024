package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor leftMotor, rightMotor;
    private final Servo mechRotation;
    private Telemetry telemetry;

    public IntakeSubsystem(Motor leftIntakeMotor, Motor rightIntakeMotor,
                           Servo mechRotation) {
        this.mechRotation = mechRotation;
        leftMotor = leftIntakeMotor;
        rightMotor = rightIntakeMotor;

        this.telemetry = null;
    }

    public IntakeSubsystem(Telemetry telemetry)
    {
        mechRotation = null;
        leftMotor = null;
        rightMotor = null;
        this.telemetry = telemetry;
    }

    public void activate() {
        leftMotor.set(0.75);
        rightMotor.set(-0.75);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void reverse() {
        leftMotor.set(-0.75);
        rightMotor.set(0.75);
    }

    double commandMechRotation = 0;

    /**
     * Grabs a object.
     */
    public void grab()
    {
        if(telemetry == null)
        commandMechRotation = (0.76);
        else
            telemetry.addLine("Grab Called");
    }

    /**
     * Releases a object.
     */
    public void release()
    {
        if(telemetry == null)
        commandMechRotation = (0);
        else
            telemetry.addLine("Release Called");
    }

    @Override
    public void periodic() 
    {
        if(mechRotation != null)
            mechRotation.setPosition(commandMechRotation);

        if(telemetry != null)
            telemetry.update();
    }
}
