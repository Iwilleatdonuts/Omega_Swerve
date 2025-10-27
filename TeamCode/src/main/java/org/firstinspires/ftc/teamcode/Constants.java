package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class Constants {

    public static final class DriveTrainConstants {

        public static final double angleKP = 0.009;//0.0042
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

        public static final class Mod0 {

            public static final String driveMotor = "drive0";
            public static final String angleServo = "angle0";
            public static final String feedback = "angleFeedback0";
            public static final double moduleOffset = 138.4364;

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
            public static final double moduleOffset = 106.8;

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
            public static final double moduleOffset = 252;

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
            public static final double moduleOffset = 244.1455;

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
            public static final SparkFunOTOS.Pose2D sensorOffset = new SparkFunOTOS.Pose2D(0, -140, 0);

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

        public static final String turretMotor = "turret";

        public static final double lowerRotationLimit = -100;
        public static final double upperRotationLimit = 100;

    }

    public static final class ShooterConstants {

        public static final String upperMotor = "upperShooter";
        public static final String lowerMotor = "lowerShooter";
        public static final String angleServo = "angleShooter";

        public static final int MAX_TRUE_RPM = 4500;

        public static final double TICKS_PER_REV = 28;
        public static final double MAX_TICKS_PER_SEC = (MAX_TRUE_RPM * TICKS_PER_REV) / 60;

//        public static final double MAX_MOTOR_RPM_THEORETICAL = 6000.0;
//        public static final double MAX_TICKS_PER_SEC = (MAX_MOTOR_RPM_THEORETICAL * TICKS_PER_REV) / 60.0;

        public static final double aimerDown = 0.13;
        public static final double aimerUp = 0.5;


        public static final double smallBig = 0.85;
        public static final double smallSmall = 0.83;

        public static final double bigBig = 0.7;
        public static final double bigMedium = 0.65;
        public static final double bigSmall = 0.6;

    }

    public static final class VisionConstants {

        public static final String aprilCameraName = "april";
        public static final Position poseCameraPosition = new Position(DistanceUnit.MM, 0, 0, 0, 0);
        public static final YawPitchRollAngles poseCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        public static final double xOffsetFromTurret = 0.09956333;
        public static final double yOffsetFromTurret = 0.02633424;
        public static final double aimingCameraYaw = 0;

    }
}
