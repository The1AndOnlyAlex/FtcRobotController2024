package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BasicSubsystem extends SubsystemBase {
    private DcMotor motor;
    private Telemetry telemetry;

    public BasicSubsystem(DcMotor motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Motor running", motor.getPower() != 0);
        telemetry.update();
    }

    public void runMotor() {
        motor.setPower(1);
    }

    public void stopMotor() {
        motor.setPower(0);
    }
}
