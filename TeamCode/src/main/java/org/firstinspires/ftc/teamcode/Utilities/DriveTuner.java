package org.firstinspires.ftc.teamcode.Utilities;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveTuner {

    public static double targetPoseIndex = 0.0;
    public static double runIntakeIndex = 0.0;
    @Config
    public static class LinearTuner{
        public static double roughLinearP = 0.5;
        public static double roughLinearI = 0.0;
        public static double roughLinearD = 0.0;
        public static double roughLinearF = 0.2;
        public static double preciseLinearP = 0.3;
        public static double preciseLinearI = 0.06;
        public static double preciseLinearD = 0.0;
        public static double preciseLinearF = 0.0295;
        public static double preciseLinearTolerance = 0.2;
        public static double preciseLinearIZone = 0.1;

    }

    @Config
    public static class AngleTuner {
        public static double roughAngleP = 0.001;
        public static double roughAngleI = 0.0;
        public static double roughAngleD = 0.0;
        public static double roughAngleF = 0.06;
        public static double preciseAngleP = 0.001;
        public static double preciseAngleI = 0.001;
        public static double preciseAngleD = 0.0;
        public static double preciseAngleF = 0.02;
        public static double preciseAngleTolerance = 30;
        public static double preciseAngleIZone = 15;
        public static double preciseAngleIMax = 0.1;
    }
}
