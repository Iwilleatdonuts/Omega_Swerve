package org.firstinspires.ftc.teamcode.Utilities;

public class HolonomicDriveController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;
    private final PIDController slowRController;

    private final Pose2D poseTolerance;

    public HolonomicDriveController(PIDController xController, PIDController yController, PIDController rController, PIDController slowRController){
        this.xController = xController;
        this.yController = yController;
        this.rController = rController;
        this.slowRController = slowRController;
        this.poseTolerance = new Pose2D(0.01, 0.01, 1); // meters + degrees
    }

    public boolean atSetpoint(Pose2D currentPose, Pose2D targetPose){
        return Math.abs(currentPose.x() - targetPose.x()) < poseTolerance.x() &&
                Math.abs(currentPose.y() - targetPose.y()) < poseTolerance.y() &&
                Math.abs(currentPose.r() - targetPose.r()) < poseTolerance.r();
    }

    public double[] calculate(Pose2D currentPose, Pose2D targetPose){
        double xOutput = xController.calculate(currentPose.x(), targetPose.x());
        double yOutput = yController.calculate(currentPose.y(), targetPose.y());

        double headingError = targetPose.r() - currentPose.r();

        double rOutput = 0;

        if(headingError < 5) {
            rOutput = -slowRController.calculate(0, headingError);
            rController.reset();
        } else {
            rOutput = -rController.calculate(0, headingError);
            slowRController.reset();
        }

        return new double[]{xOutput, yOutput, rOutput};
    }
}
