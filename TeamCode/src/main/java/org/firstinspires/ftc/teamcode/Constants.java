package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.PedroPathing.SillyCustomDrivetrain;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants();

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("intake")
            .strafeEncoder_HardwareMapName("upperShooter")
            .forwardEncoderDirection(-1)
            .strafeEncoderDirection(-1)
            .forwardTicksToInches(7.579338e-4)
            .strafeTicksToInches(7.5714066e-4)
            .forwardPodY(-1.54921)
            .strafePodX(-5.97835)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .setDrivetrain(new SillyCustomDrivetrain())
                .build();
    }

    public static final class DriveTrainConstants {

        public static final double angleKP = 0.009;//0.009
        public static final double angleKI = 0.004;
        public static final double angleKD = 0;

        //unit in MM
        //Track width is side to side modules
        //Wheelbase is front to back modules
        public static final double trackWidth = 262;
        public static final double wheelbase = 262;

        public static final double moduleHypotenuse = Math.hypot(trackWidth/2, wheelbase/2);
        public static final double widthRotation = trackWidth/moduleHypotenuse;
        public static final double lengthRotation = wheelbase/moduleHypotenuse;

        public static final double TICKS_PER_REV = 28;
        public static final double MAX_MOTOR_RPM = 6000.0;
        public static final double MAX_TICKS_PER_SEC = (MAX_MOTOR_RPM * TICKS_PER_REV) / 60.0;

        public static final double ENCODER_TICKS_PER_AZIMUTH_REVOLUTION = 60;
        public static final double funnyCouplingRatio = ENCODER_TICKS_PER_AZIMUTH_REVOLUTION / TICKS_PER_REV;

        public static final class Mod0 {

            public static final String driveMotor = "drive0";
            public static final String angleServo = "angle0";
            public static final String feedback = "angleFeedback0";
            public static final double moduleOffset = 271.5273;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.055;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(0, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod1 {

            public static final String driveMotor = "drive1";
            public static final String angleServo = "angle1";
            public static final String feedback = "angleFeedback1";
            public static final double moduleOffset = 149.7818;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.055;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(1, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod2 {

            public static final String driveMotor = "drive2";
            public static final String angleServo = "angle2";
            public static final String feedback = "angleFeedback2";
            public static final double moduleOffset = 95.5636;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.055;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(2, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod3 {

            public static final String driveMotor = "drive3";
            public static final String angleServo = "angle3";
            public static final String feedback = "angleFeedback3";
            public static final double moduleOffset = 192.4364;

            public static final double angleKP = 0;//0.02
            public static final double angleKI = 0;
            public static final double angleKD = 0;//0.001
            public static final double angleKF = 0.07;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(3, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class OTOS {

            public static final String sparkfun = "otos";

            //Currently the offset is in mm, if the offset seems weird then try setting all units to meters instead of mm
            //also if driving is backwards, change the h to 180
            public static final SparkFunOTOS.Pose2D sensorOffset = new SparkFunOTOS.Pose2D(0, -0.14, 0);

        }
    }

    public static final class IntakeConstants {

        public static final String intakeMotor = "intake";
        public static final String feederServo = "feeder";
        public static final String gateServo = "gate";

        public static final double gateClosed = 0.05;
        public static final double gateOpen = 0.25;

    }

    public static final class TurretConstants {

        public static final OmegaPose2D redTarget = new OmegaPose2D(-1.6256, 1.8288, 0);
        //change the x value to be higher absolute if not hitting properly
        public static final OmegaPose2D blueTarget = new OmegaPose2D(1.6256, 1.8288, 0);

        public static final String turretMotor = "turret";

        public static final double lowerRotationLimit = -105;
        public static final double upperRotationLimit = 105;

    }

    public static final class ShooterConstants {

        public static final String upperMotor = "upperShooter";
        public static final String lowerMotor = "lowerShooter";
        public static final String angleServo = "angleShooter";

        public static final int MAX_TRUE_RPM = 6000;

        public static final double TICKS_PER_REV = 28;
        public static final double MAX_TICKS_PER_SEC = (MAX_TRUE_RPM * TICKS_PER_REV) / 60;

//        public static final double MAX_MOTOR_RPM_THEORETICAL = 6000.0;
//        public static final double MAX_TICKS_PER_SEC = (MAX_MOTOR_RPM_THEORETICAL * TICKS_PER_REV) / 60.0;

        public static final double farAngle = 0;
        public static final double closeAngle = 1;


        public static final double smallBig = 0.85;
        public static final double smallSmall = 0.83;

        public static final double bigBig = 0.7;
        public static final double bigMedium = 0.65;
        public static final double bigSmall = 0.6;

    }

    public static final class VisionConstants {

        public static final String aprilCameraName = "april";
        public static final Position poseCameraPosition = new Position(DistanceUnit.MM, 0, 169.56645, 308.34841, 0);
        public static final YawPitchRollAngles poseCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    }

    public static final class AutoConstants {

        public static final class RedConstants {
            public static final OmegaPose2D closeStart = new OmegaPose2D(1.24088, 1.22156, 270);
            public static final OmegaPose2D farStart = new OmegaPose2D(0.48, -1.5625, 270);
            public static final OmegaPose2D closeShot = new OmegaPose2D(0.95, 0.95, 270);
            public static final OmegaPose2D mediumShot = new OmegaPose2D(0.2, 0.95, 270);
            public static final OmegaPose2D farShot = new OmegaPose2D(0.48, -1.48, 270);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(1.45, -1.45, 250);
            public static final OmegaPose2D swoopyIntake = new OmegaPose2D(1.4, -1.4, 315);
            public static final OmegaPose2D closeBallLineup = new OmegaPose2D(0.48, 0.27, 270);
            public static final OmegaPose2D mediumBallLineup = new OmegaPose2D(0.48, -0.38, 270);
            public static final OmegaPose2D farBallLineup = new OmegaPose2D(0.48, -0.99, 270);
            public static final OmegaPose2D closeBallPickup = new OmegaPose2D(1.5, 0.27, 270);
            public static final OmegaPose2D mediumBallPickup = new OmegaPose2D(1.5, -0.38, 270);
            public static final OmegaPose2D farBallPickup = new OmegaPose2D(1.7, -0.99, 270);
            public static final OmegaPose2D gateLineup = new OmegaPose2D(1.21, 0.1, 0);
            public static final OmegaPose2D gatePush = new OmegaPose2D(1.39, 0.1, 0);
            public static final OmegaPose2D gateLineupAutoTeleop = new OmegaPose2D(1, 0.1, 0);
            public static final OmegaPose2D gateLineupTeleop = new OmegaPose2D(-0.1, 1, 90);
            public static final OmegaPose2D mediumShotPositionForTeleop = new OmegaPose2D(-0.95, 0.6, 0);

        }

        public static final class BlueConstants {
            public static final OmegaPose2D closeStart = new OmegaPose2D(-1.24088, 1.22156, 90);
            public static final OmegaPose2D farStart = new OmegaPose2D(-0.48, -1.5625, 90);
            public static final OmegaPose2D closeShot = new OmegaPose2D(-0.95, 0.95, 90);
            public static final OmegaPose2D mediumShot = new OmegaPose2D(-0.2, 0.95, 90);
            public static final OmegaPose2D farShot = new OmegaPose2D(-0.48, -1.48, 90);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(-1.45, -1.45, 110);
            public static final OmegaPose2D swoopyIntake = new OmegaPose2D(-1.4, -1.4, 45);
            public static final OmegaPose2D closeBallLineup = new OmegaPose2D(-0.48, 0.27, 90);
            public static final OmegaPose2D mediumBallLineup = new OmegaPose2D(-0.48, -0.38, 90);
            public static final OmegaPose2D farBallLineup = new OmegaPose2D(-0.48, -0.99, 90);
            public static final OmegaPose2D closeBallPickup = new OmegaPose2D(-1.5, 0.27, 90);
            public static final OmegaPose2D mediumBallPickup = new OmegaPose2D(-1.5, -0.38, 90);
            public static final OmegaPose2D farBallPickup = new OmegaPose2D(-1.7, -0.99, 90);
            public static final OmegaPose2D gateLineup = new OmegaPose2D(-1.21, 0.1, 0);
            public static final OmegaPose2D gatePush = new OmegaPose2D(-1.39, 0.1, 0);
            public static final OmegaPose2D gateLineupAutoTeleop = new OmegaPose2D(-1, 0.1, 0);
            public static final OmegaPose2D gateLineupTeleop = new OmegaPose2D(0.1, 1, 270);
            public static final OmegaPose2D mediumShotPositionForTeleop = new OmegaPose2D(0.95, 0.6, 0);
        }

    }
}
