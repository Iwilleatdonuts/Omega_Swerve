package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static final String forwardEncoder_HardwareMapName = "intake";
    public static final String strafeEncoder_HardwareMapName = "upperShooter";
    public static final DcMotorSimple.Direction forwardEncoderDirection = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction strafeEncoderDirection = DcMotorSimple.Direction.REVERSE;
    public static final double forwardTicksToInches = 7.579338e-4;
    public static final double strafeTicksToInches = 7.5714066e-4;
    public static final double forwardPodY = 1.54921;
    public static final double strafePodX = -5.97835;
    public static final String IMU_HardwareMapName = "imu";
    public static final RevHubOrientationOnRobot IMU_ORIENTATION = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName(forwardEncoder_HardwareMapName)
            .strafeEncoder_HardwareMapName(strafeEncoder_HardwareMapName)
            .forwardEncoderDirection(-1)
            .strafeEncoderDirection(-1)
            .forwardTicksToInches(forwardTicksToInches)
            .strafeTicksToInches(strafeTicksToInches)
            .forwardPodY(forwardPodY)
            .strafePodX(strafePodX)
            .IMU_HardwareMapName(IMU_HardwareMapName)
            .IMU_Orientation(IMU_ORIENTATION);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .setDrivetrain(new SillyCustomDrivetrain())
                .build();
    }
    
    public static final class DriveTrainConstants {

//        public static final double angleKP = PIDTuner.PIDTuner1.kP;//0.009
//        public static final double angleKI = PIDTuner.PIDTuner1.kI;//0.004
//        public static final double angleKD = PIDTuner.PIDTuner1.kD;
//
        public static final double angleKP = 0.004;
        public static final double angleKI = 0.002;
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
            public static final double moduleOffset = 65.23;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.16;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(0, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod1 {

            public static final String driveMotor = "drive1";
            public static final String angleServo = "angle1";
            public static final String feedback = "angleFeedback1";
            public static final double moduleOffset = 187.2;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.15;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(1, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod2 {

            public static final String driveMotor = "drive2";
            public static final String angleServo = "angle2";
            public static final String feedback = "angleFeedback2";
            public static final double moduleOffset = 182.83;

            public static final double angleKP = 0;
            public static final double angleKI = 0;
            public static final double angleKD = 0;
            public static final double angleKF = 0.16;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(2, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod3 {

            public static final String driveMotor = "drive3";
            public static final String angleServo = "angle3";
            public static final String feedback = "angleFeedback3";
            public static final double moduleOffset = 102.7;

            public static final double angleKP = 0;//0.02
            public static final double angleKI = 0;
            public static final double angleKD = 0;//0.001
            public static final double angleKF = 0.13;

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

        public static final OmegaPose2D redTarget = new OmegaPose2D(-1.8288, 1.8288, 0);
        //change the x value to be higher absolute if not hitting properly
        public static final OmegaPose2D blueTarget = new OmegaPose2D(1.8288, 1.8288, 0);

        public static final String turretMotor = "turret";

        public static final double lowerRotationLimit = -135;
        public static final double upperRotationLimit = 135;

    }

    public static final class ShooterConstants {

        public static final String upperMotor = "upperShooter";
        public static final String lowerMotor = "lowerShooter";
        public static final String angleServo = "angleShooter";

        public static final int MAX_TRUE_RPM = 6000;

        public static final double TICKS_PER_REV = 28;
        public static final double MAX_TICKS_PER_SEC = (MAX_TRUE_RPM * TICKS_PER_REV) / 60;

        public static final double farAngle = 0;
        public static final double closeAngle = 1;

    }

    public static final class VisionConstants {

        public static final String aprilCameraName = "april";
        public static final Position poseCameraPosition = new Position(DistanceUnit.MM, 0, 169.56645, 308.34841, 0);
        public static final YawPitchRollAngles poseCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    }

    public static final class NewAutoConstants {

        public static final class RedConstants {

            //Starts and shots
            public static final OmegaPose2D closeStart = new OmegaPose2D(1.23063, 1.20049, 270);
            public static final OmegaPose2D farStart = new OmegaPose2D(0.40491, -1.58721, 270);
            public static final OmegaPose2D closeShot = new OmegaPose2D(0.56799, 0.40199, 260);
            public static final OmegaPose2D farShot = new OmegaPose2D(0.40491, -1.47621, 270);
            public static final OmegaPose2D finalCloseShot = new OmegaPose2D(0.39055, 1.00344, 180);
            public static final OmegaPose2D finalCloseShotTeleopPose = new OmegaPose2D(-1.00344, 0.39055, 270);

            //Spike Marks
            public static final OmegaPose2D firstSpikeLineup = new OmegaPose2D(0.70389, 0.30335, 270);
            public static final OmegaPose2D secondSpikeLineup = new OmegaPose2D(0.70389, -0.30665, 270);
            public static final OmegaPose2D thirdSpikeLineup = new OmegaPose2D(0.70389, -0.88965, 270);
            public static final OmegaPose2D firstSpikePickup = new OmegaPose2D(1.31419, 0.30335, 270);
            public static final OmegaPose2D secondSpikePickup = new OmegaPose2D(1.31419, -0.30665, 270);
            public static final OmegaPose2D thirdSpikePickup = new OmegaPose2D(1.47489, -0.88965, 270);

            //Support balls
            public static final OmegaPose2D cornerLineup = new OmegaPose2D(1.36233, -1.55573, 270);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(1.47489, -1.55573, 270);
            public static final OmegaPose2D firstSupportLineup = new OmegaPose2D(1.36233, -1, 270);
            public static final OmegaPose2D firstSupportPickup = new OmegaPose2D(1.47489, -1, 270);
            public static final OmegaPose2D secondSupportLineup = new OmegaPose2D(1.36233, -0.55, 270);
            public static final OmegaPose2D secondSupportPickup = new OmegaPose2D(1.47489, -0.55, 270);

        }

        public static final class BlueConstants {

            //Starts and shots
            public static final OmegaPose2D closeStart = new OmegaPose2D(-1.23063, 1.20049, 90);
            public static final OmegaPose2D farStart = new OmegaPose2D(-0.40491, -1.58721, 90);
            public static final OmegaPose2D closeShot = new OmegaPose2D(-0.56799, 0.40199, 100);
            public static final OmegaPose2D farShot = new OmegaPose2D(-0.40491, -1.47621, 90);
            public static final OmegaPose2D finalCloseShot = new OmegaPose2D(-0.39055, 1.00344, 180);
            public static final OmegaPose2D finalCloseShotTeleopPose = new OmegaPose2D(1.00344, 0.39055, 90);

            //Spike Mark
            public static final OmegaPose2D firstSpikeLineup = new OmegaPose2D(-0.70389, 0.30335, 90);
            public static final OmegaPose2D secondSpikeLineup = new OmegaPose2D(-0.70389, -0.30665, 90);
            public static final OmegaPose2D thirdSpikeLineup = new OmegaPose2D(-0.70389, -0.88965, 90);
            public static final OmegaPose2D firstSpikePickup = new OmegaPose2D(-1.31419, 0.30335, 90);
            public static final OmegaPose2D secondSpikePickup = new OmegaPose2D(-1.31419, -0.30665, 90);
            public static final OmegaPose2D thirdSpikePickup = new OmegaPose2D(-1.47489, -0.88965, 90);

            //Support balls
            public static final OmegaPose2D cornerLineup = new OmegaPose2D(-1.36233, -1.55573, 90);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(-1.47489, -1.55573, 90);
            public static final OmegaPose2D firstSupportLineup = new OmegaPose2D(-1.36233, -1, 90);
            public static final OmegaPose2D firstSupportPickup = new OmegaPose2D(-1.47489, -1, 90);
            public static final OmegaPose2D secondSupportLineup = new OmegaPose2D(-1.36233, -0.55, 90);
            public static final OmegaPose2D secondSupportPickup = new OmegaPose2D(-1.47489, -0.55, 90);
        }

    }

    public static final class OldAutoConstants {

        public static final class RedConstants {
            public static final OmegaPose2D closeStart = new OmegaPose2D(1.24088, 1.2, 270);
            public static final OmegaPose2D farStart = new OmegaPose2D(0.504, -1.6029, 270);
            public static final OmegaPose2D closeShot = new OmegaPose2D(0.95, 0.95, 270);
            public static final OmegaPose2D mediumShot = new OmegaPose2D(0.2, 0.95, 270);
            public static final OmegaPose2D farShot = new OmegaPose2D(0.504, -1.55, 270);
            public static final OmegaPose2D cornerLineup = new OmegaPose2D(1.484, -1.3069, 240);
            public static final OmegaPose2D cornerMidPose = new OmegaPose2D(1.471, -1.5829, 240);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(1.471, -1.6029, 270);
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
            public static final OmegaPose2D farStart = new OmegaPose2D(-0.504, -1.6029, 90);
            public static final OmegaPose2D closeShot = new OmegaPose2D(-0.95, 0.95, 90);
            public static final OmegaPose2D mediumShot = new OmegaPose2D(-0.2, 0.95, 90);
            public static final OmegaPose2D farShot = new OmegaPose2D(-0.504, -1.55, 90);
            public static final OmegaPose2D cornerLineup = new OmegaPose2D(-1.484, -1.3069, 120);
            public static final OmegaPose2D cornerMidPose = new OmegaPose2D(-1.471, -1.5829, 120);
            public static final OmegaPose2D cornerPickup = new OmegaPose2D(-1.471, -1.6029, 90);
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
