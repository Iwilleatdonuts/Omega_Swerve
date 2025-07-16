package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class Constants {

    public static final class DriveTrainConstants {

        public static final double angleKP = 0.025;
        public static final double angleKI = 0.01;
        public static final double angleKD = 0;
        public static final double angleKF = 0.0005;

        //unit in MM
        //Track width is side to side modules
        //Wheelbase is front to back modules
        public static final double trackWidth = 286.40000;
        public static final double wheelbase = 286.40000;

        public static final double moduleHypotenuse = Math.hypot(trackWidth/2, wheelbase/2);
        public static final double widthRotation = trackWidth/moduleHypotenuse;
        public static final double lengthRotation = wheelbase/moduleHypotenuse;

        public static final class Mod0 {

            public static final String driveMotor = "drive0";
            public static final String angleServo = "angle0";
            public static final String feedback = "angleFeedback0";
            public static final double moduleOffset = 107.5636;

            public static final double angleKP = 0.02;
            public static final double angleKI = 0.015;
            public static final double angleKD = 0;
            public static final double angleKF = 0.0005;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(0, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod1 {

            public static final String driveMotor = "drive1";
            public static final String angleServo = "angle1";
            public static final String feedback = "angleFeedback1";
            public static final double moduleOffset = 269.8909;

            public static final double angleKP = 0.02;
            public static final double angleKI = 0.015;
            public static final double angleKD = 0;
            public static final double angleKF = 0.0005;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(1, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod2 {

            public static final String driveMotor = "drive2";
            public static final String angleServo = "angle2";
            public static final String feedback = "angleFeedback2";
            public static final double moduleOffset = 178.1455;

            public static final double angleKP = 0.02;
            public static final double angleKI = 0.015;
            public static final double angleKD = 0;
            public static final double angleKF = 0.0005;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(2, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class Mod3 {

            public static final String driveMotor = "drive3";
            public static final String angleServo = "angle3";
            public static final String feedback = "angleFeedback3";
            public static final double moduleOffset = 133.8545;

            public static final double angleKP = 0.02;
            public static final double angleKI = 0.015;
            public static final double angleKD = 0;
            public static final double angleKF = 0.0005;

            public  static final SwerveModuleConstants modConstants = new SwerveModuleConstants(3, driveMotor, angleServo, feedback, moduleOffset, angleKP, angleKI, angleKD, angleKF);

        }

        public static final class OTOS {

            public static final String sparkfun = "otos";

            //Currently the offset is in mm, if the offset seems weird then try setting all units to meters instead of mm
            //also if driving is backwards, change the h to 180
            public static final SparkFunOTOS.Pose2D sensorOffset = new SparkFunOTOS.Pose2D(0, -48, 0);

        }
    }
}
