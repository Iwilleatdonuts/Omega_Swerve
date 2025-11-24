package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.WaypointFollower;

public class AutoWaypointIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final OTOSSensor s_Sparky;

    private OmegaPose2D finalTarget;
    private OmegaPose2D pickupTarget;

    private final AutoDriveController driveController;
    private final WaypointFollower waypointFollower;

    private final boolean areWeWinners;
    private int targetRow;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double[] outputs = new double[3];
    private OmegaPose2D[] targetPositions = new OmegaPose2D[3];

    public AutoWaypointIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, OTOSSensor s_Sparky, EZTelemetry telem, boolean areWeWinners, int targetRow){

        this.targetRow = targetRow;

        this.areWeWinners = areWeWinners;

        this.telem = telem;

       setIntakeRow();

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Sparky = s_Sparky;

        driveController = new AutoDriveController();
        waypointFollower = new WaypointFollower(driveController);
    }

    public void reset(int newRow){
        driveController.reset();
        waypointFollower.resetWaypointFollower();
        isFinished = false;
        phase = 0;
        targetRow = newRow;
        setIntakeRow();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();

        s_Feeder.closeGate();
        s_Feeder.setFeederSpeed(0);
        s_Intake.setSpeed(1);

        switch(phase) {
            case 0:
                    driveController.reset();
                waypointFollower.resetWaypointFollower();
                    phase++;
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);
                outputs = waypointFollower.getWaypointOutputs(currentPose, targetPositions);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

                if(isAtRoughSetpoint()) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    driveController.reset();
                    driveController.setTargetPose(pickupTarget);
                    isFinished = true;
                    phase++;
                }

                break;
        }
    }

    public boolean isAtSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - finalTarget.x());
        double yError = Math.abs(s_Sparky.getPose().y() - finalTarget.y());
        double rError = Math.abs(s_Sparky.getHeading() - finalTarget.r());

        return xError < 0.02 && yError < 0.02 && rError < 3;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - finalTarget.x());
        double yError = Math.abs(s_Sparky.getPose().y() - finalTarget.y());
        double rError = Math.abs(s_Sparky.getHeading() - finalTarget.r());

        return xError < 0.6 && yError < 0.06 && rError < 6;
    }

    public boolean isFinished() {
        return isFinished;
    }

    private void setIntakeRow() {
        if(areWeWinners) {
            switch(targetRow) {
                case 1:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.RedConstants.closeBallLineup.x()-0.4,
                                    Constants.AutoConstants.RedConstants.closeBallLineup.y()+0.5,
                                    Constants.AutoConstants.RedConstants.closeBallLineup.r()),
                            Constants.AutoConstants.RedConstants.closeBallLineup,
                            Constants.AutoConstants.RedConstants.closeBallPickup
                    };
                    break;
                case 2:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.RedConstants.mediumBallLineup.x()-0.4,
                                    Constants.AutoConstants.RedConstants.mediumBallLineup.y()+0.5,
                                    Constants.AutoConstants.RedConstants.mediumBallLineup.r()),
                            Constants.AutoConstants.RedConstants.mediumBallLineup,
                            Constants.AutoConstants.RedConstants.mediumBallPickup
                    };
                    break;
                case 3:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.RedConstants.farBallLineup.x()-0.4,
                                    Constants.AutoConstants.RedConstants.farBallLineup.y()+0.5,
                                    Constants.AutoConstants.RedConstants.farBallLineup.r()),
                            Constants.AutoConstants.RedConstants.farBallLineup,
                            Constants.AutoConstants.RedConstants.farBallPickup
                    };
                    break;
            }
        } else {

            switch(targetRow) {
                case 1:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.BlueConstants.closeBallLineup.x()+0.4,
                                    Constants.AutoConstants.BlueConstants.closeBallLineup.y()+0.5,
                                    Constants.AutoConstants.BlueConstants.closeBallLineup.r()),
                            Constants.AutoConstants.BlueConstants.closeBallLineup,
                            Constants.AutoConstants.BlueConstants.closeBallPickup
                    };
                    break;
                case 2:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.BlueConstants.mediumBallLineup.x()+0.4,
                                    Constants.AutoConstants.BlueConstants.mediumBallLineup.y()+0.5,
                                    Constants.AutoConstants.BlueConstants.mediumBallLineup.r()),
                            Constants.AutoConstants.BlueConstants.mediumBallLineup,
                            Constants.AutoConstants.BlueConstants.mediumBallPickup
                    };
                    break;
                case 3:
                    targetPositions = new OmegaPose2D[]{
                            new OmegaPose2D(
                                    Constants.AutoConstants.BlueConstants.farBallLineup.x()+0.4,
                                    Constants.AutoConstants.BlueConstants.farBallLineup.y()+0.5,
                                    Constants.AutoConstants.BlueConstants.farBallLineup.r()),
                            Constants.AutoConstants.BlueConstants.farBallLineup,
                            Constants.AutoConstants.BlueConstants.farBallPickup
                    };
                    break;
            }
        }

        finalTarget = targetPositions[2];
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
