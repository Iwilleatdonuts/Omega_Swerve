package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;

import static org.firstinspires.ftc.teamcode.Utilities.DriveTuner.AngleTuner.*;

import static org.firstinspires.ftc.teamcode.Utilities.DriveTuner.LinearTuner.*;

public class AutoDriveController {

    private PIDController roughXController;
    private PIDController roughYController;

    private PIDController preciseXController;
    private PIDController preciseYController;

    private PIDController roughAngleController;
    private PIDController preciseAngleController;

    private OmegaPose2D targetPose;
    private OmegaPose2D currentPose;

    private double roughXFF;
    private double preciseXFF;

    private double roughYFF;
    private double preciseYFF;

    public AutoDriveController() {

        initializeControllers();

        targetPose = new OmegaPose2D(0, 0, 0);
        currentPose = new OmegaPose2D(0, 0, 0);

    }

    public void reset() {
        roughXController.reset();
        roughYController.reset();
        preciseXController.reset();
        preciseYController.reset();
        roughAngleController.reset();
        preciseAngleController.reset();
    }

    public double[] getOutputs() {

        double[] outputs = new double[3];

        double xOutput;
        double yOutput;

        if(getXError() < 0.01) {
            xOutput = 0;
        } else if(getXError() > preciseLinearTolerance) {
            xOutput = roughXController.calculate(currentPose.x(), targetPose.x());
            xOutput += (Math.signum(xOutput) * roughXFF);
        } else {
            xOutput = preciseXController.calculate(currentPose.x(), targetPose.x());
            xOutput += (Math.signum(xOutput) * preciseXFF);
        }

        if(getYError() < 0.01) {
            yOutput = 0;
        } else if(getYError() > preciseLinearTolerance) {
            yOutput = roughYController.calculate(currentPose.y(), targetPose.y());
            yOutput += (Math.signum(yOutput) * roughYFF);
        } else {
            yOutput = preciseYController.calculate(currentPose.y(), targetPose.y());
            yOutput += (Math.signum(yOutput) * preciseYFF);
        }

        double rotationOutput;

        double currentHeading = currentPose.r();
        if(getRError() < 1) {
            rotationOutput = 0;
        } else if(getRError() > preciseAngleTolerance) {
            rotationOutput = -roughAngleController.calculate(currentHeading, targetPose.r());
            rotationOutput += Math.signum(rotationOutput) * roughAngleF;
        } else {
            rotationOutput = -preciseAngleController.calculate(currentHeading, targetPose.r());
            rotationOutput += Math.signum(rotationOutput) * preciseAngleF;
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

    public void initializeControllers() {

        roughXController = new PIDController(roughLinearP, roughLinearI, roughLinearD);
        roughYController = new PIDController(roughLinearP, roughLinearI, roughLinearD);
        preciseXController = new PIDController(preciseLinearP, preciseLinearI, preciseLinearD);
        preciseYController = new PIDController(preciseLinearP, preciseLinearI, preciseLinearD);

        preciseXController.setIZone(preciseLinearIZone);
        preciseYController.setIZone(preciseLinearIZone);

        roughXFF = roughLinearF;
        preciseXFF = preciseLinearF;
        roughYFF = roughLinearF;
        preciseYFF = preciseLinearF;

        roughAngleController = new PIDController(roughAngleP, roughAngleI, roughAngleD);
        preciseAngleController = new PIDController(preciseAngleP, preciseAngleI, preciseAngleD);

        roughAngleController.enableContinuousInput(0, 360);
        preciseAngleController.enableContinuousInput(0, 360);

        preciseAngleController.setIZone(preciseAngleIZone);
        preciseAngleController.setIntegratorRange(-preciseAngleIMax, preciseAngleIMax);

//        staticAnglePID = new PIDController(0.006, 0.02, 0.00015);
//        staticAnglePID.setIZone(40);
//        staticAnglePID.enableContinuousInput(0, 360);
//
//        dynamicAnglePID = new PIDController(0.0035, 0.015, 0.0001);
//        dynamicAnglePID.setIZone(40);
//        dynamicAnglePID.enableContinuousInput(0, 360);
    }

}
