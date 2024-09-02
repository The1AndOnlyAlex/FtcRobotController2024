package Config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

    static final double FEET_PER_METER = 3.28084;

    //COUNTS_PER_MOTOR_REV = 28;    //UltraPlanetary Gearbox Kit & HD Hex Motor
    //DRIVE_GEAR_REDUCTION = 20;   //gear ratio
    public static double TICKS_PER_REV = 537.7;//537.7;//28*19.2;// //383.6;
    public static double WHEEL_DIAMETER = 0.096;//3.5inch, 90mm //0.1;
    public static double DISTANCE_PER_PULSE = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
    public static double TRACK_WIDTH = 0.4;//0.4572;

    public static double MAX_VELOCITY = 1.5;
    public static double MAX_ACCELERATION = 1.5;//0.5;//1.5;

    // DiffDrive Ctrl Coeff
    public static double B = 2.0;
    public static double ZETA = 0.7;
    // end DiffDrive Ctrl Coeff
    public static double TRAJECTORY_MAX_VELOCITY = 1.0;//1.5;//0.5;

    public static MecanumDriveKinematics kinematicsWPI =
                new MecanumDriveKinematics (
                        new edu.wpi.first.math.geometry.Translation2d(0.2, 0.21),
                        new edu.wpi.first.math.geometry.Translation2d(0.2, -0.21),
                        new edu.wpi.first.math.geometry.Translation2d(-0.2, 0.21),
                        new edu.wpi.first.math.geometry.Translation2d(-0.2, -0.21)
        );

    public static Transform3d CamUnderRobot = new Transform3d(-0.1, +0.21, -0.04,
            new Rotation3d(0,0, Units.degreesToRadians(180)));

    public static Transform3d CamToRobot = new Transform3d(-0.19f, 0.0f, 0.14,
            new Rotation3d(0,0, Units.degreesToRadians(180)));

    public static final int RESOLUTION_WIDTH = 640;
    public static final int RESOLUTION_HEIGHT = 480;


    public static final int RESOLUTION_WIDTH1 = 1280;
    public static final int RESOLUTION_HEIGHT1 = 720;
    /*
    Focals (pixels) - Fx: 735.337 Fy: 735.337
    Optical center - Cx: 641.344 Cy: 377.929
    Radial distortion (Brown's Model)
    K1: 0.0551262 K2: -0.0942521 K3: 0.0287788
    P1: 0.000520371 P2: -0.000457627
    Skew: 0
     */




}
