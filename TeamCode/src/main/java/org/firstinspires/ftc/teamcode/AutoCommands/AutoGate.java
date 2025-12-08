package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoGate {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;
    private boolean goingToTeleop;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    public AutoGate(Swerve s_Swerve, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Lemon = s_Lemon;

        driveController = new AutoDriveController();
    }

    public void reset(boolean teleopNext){
        goingToTeleop = teleopNext;
        if(goingToTeleop) {
            targetPosition = areWeWinners? Constants.AutoConstants.RedConstants.gateLineupAutoTeleop : Constants.AutoConstants.BlueConstants.gateLineupAutoTeleop;
        } else {
            targetPosition = areWeWinners? Constants.AutoConstants.RedConstants.gateLineup : Constants.AutoConstants.BlueConstants.gateLineup;
        }
        isFinished = false;
        phase = 0;
        driveController.reset();
        driveController.setTargetPose(targetPosition);
    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();

        if(goingToTeleop) {

        }

        switch(phase) {
            case 0:
                if(areWeWinners && currentPose.x() > 1.21) {
                    s_Swerve.drive(-0.8, 0, 0, true);
                } else if (!areWeWinners && currentPose.x() < -1.21) {
                    s_Swerve.drive(0.8, 0, 0, true);
                } else {
                    driveController.reset();
                    phase++;
                }
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);
                double[] outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint()) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    phase++;
                }

                break;
            case 2:

                s_Swerve.drive(0.4, 0, 0, true);

                if(System.nanoTime() - timestamp > 1e9 || goingToTeleop) {
                    s_Swerve.stop();
                    isFinished = true;
                }

                break;
        }
    }

    public boolean isAtSetpoint(){
        double xError = Math.abs(s_Lemon.getCurrentPose().x() - targetPosition.x());
        double yError = Math.abs(s_Lemon.getCurrentPose().y() - targetPosition.y());
        double rError = Math.abs(s_Lemon.getHeading() - targetPosition.r());

        return xError < 0.02 && yError < 0.02 && rError < 3;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Lemon.getCurrentPose().x() - targetPosition.x());
        double yError = Math.abs(s_Lemon.getCurrentPose().y() - targetPosition.y());
        double rError = Math.abs(s_Lemon.getHeading() - targetPosition.r());

        return xError < 0.1 && yError < 0.1 && rError < 4;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }
}
