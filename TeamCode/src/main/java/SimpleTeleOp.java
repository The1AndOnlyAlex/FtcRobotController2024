//package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import commands.DriveCommand;
import commands.RunMotorCommand;
import commands.StopMotorCommand;
import subsystems.BasicSubsystem;
import subsystems.IntakeSubsystem;
import subsystems.TankDriveSubsystem;

@TeleOp
public class SimpleTeleOp extends CommandOpMode {

    private BasicSubsystem subsystem;

    // define our constants
    static final double TRACKWIDTH = 13.7;
    static final double TICKS_TO_INCHES = 15.3;
    static final double CENTER_WHEEL_OFFSET = 2.4;


    // create our encoders
    //MotorEx encoderLeft, encoderRight, encoderPerp;
    MotorEx encoderLeft = new MotorEx(hardwareMap, "left_encoder");
    MotorEx encoderRight = new MotorEx(hardwareMap, "right_encoder");
    MotorEx encoderPerp = new MotorEx(hardwareMap, "center_encoder");



    // create the odometry object
    HolonomicOdometry holOdom;

    // create the odometry subsystem
    OdometrySubsystem odometry;

    @Override
    public void initialize() {
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);

        GamepadEx gamepad = new GamepadEx(gamepad1);

        TankDriveSubsystem driveSystem = new TankDriveSubsystem(
                new Motor(hardwareMap, "left_drive_motor"),
                new Motor(hardwareMap, "right_drive_motor"),
                gamepad
        );

        IntakeSubsystem intake = new IntakeSubsystem(
                new Motor(hardwareMap, "left_intake_motor"),
                new Motor(hardwareMap, "right_intake_motor"),
                null//Servo mechRotation
        );

        subsystem = new BasicSubsystem(hardwareMap.get(DcMotor.class, "motor"), telemetry);

        // bindings
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new RunMotorCommand(subsystem));
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new StopMotorCommand(subsystem));

        // button bindings for the intake
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(intake::activate, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        // sets the default command to the drive command so that it is always looking
        // at the value on the joysticks
        driveSystem.setDefaultCommand(new DriveCommand(driveSystem));

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
    }
}
