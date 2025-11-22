package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;

public class AutoDriveController {

    private final PIDController xController;
    private final PIDController yController;

    private final PIDController slowXController;
    private final PIDController slowYController;

    private final PIDController staticAnglePID;
    private final PIDController dynamicAnglePID;

    private OmegaPose2D targetPose;
    private OmegaPose2D currentPose;

    public AutoDriveController() {

        xController = new PIDController(2.2, 0, 0.05);
        yController = new PIDController(2.2, 0, 0.05);
        xController.setIZone(0.5);
        yController.setIZone(0.5);

        slowXController = new PIDController(0.5, 0.08, 0.02);
        slowYController = new PIDController(0.5, 0.08, 0.02);
        slowXController.setIZone(0.5);
        slowYController.setIZone(0.5);

        staticAnglePID = new PIDController(0.006, 0.02, 0.00015);
//        staticAnglePID = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        staticAnglePID.setIZone(40);
        staticAnglePID.enableContinuousInput(0, 360);

        dynamicAnglePID = new PIDController(0.0035, 0.015, 0.0001);
//        dynamicAnglePID = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);
        dynamicAnglePID.setIZone(40);
        dynamicAnglePID.enableContinuousInput(0, 360);

        targetPose = new OmegaPose2D(0, 0, 0);
        currentPose = new OmegaPose2D(0, 0, 0);

    }

    public void reset() {
        xController.reset();
        yController.reset();
        staticAnglePID.reset();
        dynamicAnglePID.reset();
    }

    public double[] getOutputs() {

        double[] outputs = new double[3];

        double xOutput = xController.calculate(currentPose.x(), targetPose.x());
        double yOutput = yController.calculate(currentPose.y(), targetPose.y());

        double rotationOutput = 0;

        double currentHeading = currentPose.r();
        if(Math.abs(currentHeading - targetPose.r()) > 0.5) {
            if(xOutput != 0 || yOutput != 0) {
                rotationOutput = -dynamicAnglePID.calculate(currentHeading, targetPose.r());
            } else {
                rotationOutput = -staticAnglePID.calculate(currentHeading, targetPose.r());
            }
        }

        outputs[0] = xOutput;
        outputs[1] = yOutput;
        outputs[2] = rotationOutput;

        return outputs;
    }

    public double[] getSlowOutputs() {

        double[] outputs = new double[3];

        double xOutput = slowXController.calculate(currentPose.x(), targetPose.x());
        double yOutput = slowYController.calculate(currentPose.y(), targetPose.y());

        double rotationOutput = 0;

        double currentHeading = currentPose.r();
        if(Math.abs(currentHeading - targetPose.r()) > 0.5) {
            if(xOutput != 0 || yOutput != 0) {
                rotationOutput = -dynamicAnglePID.calculate(currentHeading, targetPose.r());
            } else {
                rotationOutput = -staticAnglePID.calculate(currentHeading, targetPose.r());
            }
        }

        outputs[0] = xOutput;
        outputs[1] = yOutput;
        outputs[2] = rotationOutput;

        return outputs;
    }

    public void setTargetPose(OmegaPose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void updateCurrentPose(OmegaPose2D currentPose) {
        this.currentPose = currentPose;
    }

    public double getXError() {
        return Math.abs(targetPose.x() - currentPose.x());
    }

    public double getYError() {
        return Math.abs(targetPose.y() - currentPose.y());
    }

    public double getRError() {
        return Math.abs(targetPose.r() - currentPose.r());
    }

    public boolean isAtSetpoint() {
        return getXError() < 0.03 && getYError() < 0.03 && getRError() < 1;
    }

    public boolean isAtRoughSetpoint() {
        return getXError() < 0.1 && getYError() < 0.1 && getRError() < 10;
    }

}
