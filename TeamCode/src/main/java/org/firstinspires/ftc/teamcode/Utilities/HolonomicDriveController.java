package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class HolonomicDriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;

    private final SparkFunOTOS.Pose2D poseTolerance;

    public HolonomicDriveController(PIDController xController, PIDController yController, PIDController rController){
        this.xController = xController;
        this.yController = yController;
        this.rController = rController;
        this.poseTolerance = new SparkFunOTOS.Pose2D(0.01, 0.01, 1); // meters + degrees
    }

    public boolean atSetpoint(SparkFunOTOS.Pose2D currentPose, SparkFunOTOS.Pose2D targetPose){
        return Math.abs(currentPose.x - targetPose.x) < poseTolerance.x &&
                Math.abs(currentPose.y - targetPose.y) < poseTolerance.y &&
                Math.abs(currentPose.h - targetPose.h) < poseTolerance.h;
    }

    public double[] calculate(SparkFunOTOS.Pose2D currentPose, SparkFunOTOS.Pose2D targetPose){
        double xOutput = xController.calculate(currentPose.x, targetPose.x);
        double yOutput = yController.calculate(currentPose.y, targetPose.y);

        double headingError = targetPose.h - currentPose.h;
        headingError = ((headingError + 180) % 360) - 180;
        double rOutput = rController.calculate(0, headingError);

        return new double[]{xOutput, yOutput, rOutput};
    }
}
