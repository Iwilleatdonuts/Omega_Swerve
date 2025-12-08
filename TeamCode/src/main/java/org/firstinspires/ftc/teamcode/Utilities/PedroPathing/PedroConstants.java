package org.firstinspires.ftc.teamcode.Utilities.PedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.1542);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("intake")
            .strafeEncoder_HardwareMapName("upperShooter")
            .forwardEncoderDirection(-1)
            .strafeEncoderDirection(-1)
            .forwardTicksToInches(7.6031718e-4)
            .strafeTicksToInches(7.6205e-4)
            .forwardPodY(-1.54921)
            .strafePodX(-5.97835)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));;

            public static MecanumConstants mecanumConstants = new MecanumConstants()
                    .maxPower(1)
                    .rightFrontMotorName("drive1")
                    .rightRearMotorName("drive3")
                    .leftRearMotorName("drive2")
                    .leftFrontMotorName("drive0")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    ;

    public static Follower createFollower(HardwareMap hardwareMap, Swerve swerve) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(new TwoWheelLocalizer(hardwareMap, localizerConstants))
                .setDrivetrain(swerve)
//                .setDrivetrain(new Mecanum(hardwareMap, mecanumConstants))
//                .setDrivetrain(new CustomSwerveDrivetrain(hardwareMap, new CustomSwerveConstants()))
                .build();
    }
}
