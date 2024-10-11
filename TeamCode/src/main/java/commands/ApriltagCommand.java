package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import subsystems.MecanumDriveSubsystem;

public class ApriltagCommand extends CommandBase {
    private MecanumDriveSubsystem subsystem;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;

    AprilTagProcessor apriltagCam;

    Telemetry telemetry;

    public ApriltagCommand(MecanumDriveSubsystem subsystem, AprilTagProcessor aprilTag, Telemetry telemetry) {
        this.subsystem = subsystem;
        this.aprilTag = aprilTag;
        this.telemetry = telemetry;

        addRequirements(subsystem);
        //initAprilTag();
    }



    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
//    private boolean    setManualExposure(int exposureMS, int gain) {
//        // Ensure Vision Portal has been setup.
//        if (visionPortal == null) {
//            return false;
//        }
//
//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested())
//        {
//            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//
//            // Set Gain.
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//            return (true);
//        } else {
//            return (false);
//        }
//    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
//    private void getCameraSetting() {
//        // Ensure Vision Portal has been setup.
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Wait for the camera to be open
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Get camera control values unless we are stopping.
//        if (!isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
//            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
//
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            minGain = gainControl.getMinGain();
//            maxGain = gainControl.getMaxGain();
//        }
//    }

    @Override
    public void initialize() {
        //subsystem.runMotor();
    }

    @Override
    public void execute() {
        telemetryAprilTag();
    }
    
    // @Override
    // public void end(boolean interrupted) {
    //   m_timer.stop();
    // }

//    @Override
//    public boolean isFinished() {
//        return true;
//    }
}
