import com.arcrobotics.ftclib.command.CommandOpMode;
import commands.Auto3PushCommand;
import subsystems.IntakeSubsystem;
import subsystems.MecanumDriveSubsystem;
import util.DashServer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

@Autonomous
public class AutoOpmodeMecanumPathPlan extends CommandOpMode
{
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize()
    {
        //telemetry.setMsTransmissionInterval(11);

        intakeSubsystem = new IntakeSubsystem(telemetry);

        mecanumDriveSubsystem = new MecanumDriveSubsystem(
                hardwareMap,
                new edu.wpi.first.math.geometry.Pose2d(),
                telemetry
        );

        schedule(new Auto3PushCommand(
                mecanumDriveSubsystem,
                intakeSubsystem
        ));
    }


    ////// DO NOT MODIFY THIS FUNCTION UNLESS YOU GET CONFIRMED!!!
    private int FrameCounter = 0;
    @Override
    public void runOpMode() {
        LynxModule controlHub  = hardwareMap.get(LynxModule.class, "Control Hub");
        DashServer.Init();
        boolean connected = false;
        do {
            connected = DashServer.Connect();
            connected |= DashServer.AddData("time", FrameCounter);
            sleep(1);
        } while (!connected);

        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            run();
            DashServer.AddData("time", FrameCounter++);
            DashServer.AddData("busVoltage",
                    controlHub .getInputVoltage(VoltageUnit.VOLTS));
            DashServer.DashData();
            sleep(10);
        }
        reset();
        mecanumDriveSubsystem.subsystemExit();
        DashServer.Close();
    }
}
