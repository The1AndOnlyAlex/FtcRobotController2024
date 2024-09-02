package Config;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class ApriltagsFieldData {

    public static List<AprilTag> apriltags = Arrays.asList(
        new AprilTag(1, new Pose3d(0, 0, 0, new Rotation3d(0,0,0))),
        new AprilTag(2, new Pose3d(3, 3, 3, new Rotation3d(1,1,1)))

    );
    public static double fieldLength = 10;
    public static double fieldWidth = 10;

    //A tag library contains metadata about tags such as their ID, name, size, and 6DOF position on the field
    //AprilTagProcessor.Builder	setTagLibrary​(AprilTagLibrary tagLibrary)
    //Inform the detector about known tags. The tag library is used to allow solving for 6DOF pose, based on 
    //the physical size of the tag. Tags which are not in the library will not have their pose solved for

    //AprilTagLibrary.Builder.addTag​(tag1);
    

    /*Add a tag to this tag library
Parameters:
id - the ID of the tag
name - a text name for the tag
tagsize - the physical size of the tag in the real world (measured black edge to black edge)
fieldPosition - a vector describing the tag's 3d translation on the field
distanceUnit - the units used for size and fieldPosition
fieldOrientation - a quaternion describing the tag's orientation on the field */
    public static AprilTagMetadata tag_42 = new AprilTagMetadata( // with timestamp
        42,//int id,
        "42",//String name,
        0.13,//double tagsize,
        new org.firstinspires.ftc.robotcore.external.matrices.VectorF(0,0,0),// fieldPosition,
        DistanceUnit.METER, //distanceUnit,
        new org.firstinspires.ftc.robotcore.external.navigation.Quaternion(0,0,0,0,0)// fieldOrientation
        );

    public static AprilTagMetadata tag_2 = new AprilTagMetadata( // with timestamp
            2,//int id,
            "2",//String name,
            0.189,//double tagsize,
            new org.firstinspires.ftc.robotcore.external.matrices.VectorF(0,0,0.07f),// fieldPosition,
            DistanceUnit.METER, //distanceUnit,
            new org.firstinspires.ftc.robotcore.external.navigation.Quaternion(0,0,0,0,0)// fieldOrientation
    );

}
