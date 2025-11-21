package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final OTOSSensor s_Sparky;

    private OmegaPose2D lineupTarget;
    private OmegaPose2D pickupTarget;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;
    private int targetRow;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double[] outputs = new double[3];

    public AutoIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, OTOSSensor s_Sparky, EZTelemetry telem, boolean areWeWinners, int targetRow){

        this.targetRow = targetRow;

        this.areWeWinners = areWeWinners;

        this.telem = telem;

       setIntakeRow();

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Sparky = s_Sparky;

        driveController = new AutoDriveController();
    }

    public void reset(int newRow){
        isFinished = false;
        phase = 0;
        targetRow = newRow;
        setIntakeRow();
        driveController.reset();
        driveController.setTargetPose(lineupTarget);
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();

        s_Feeder.closeGate();
        s_Feeder.setFeederSpeed(0);

        switch(phase) {
            case 0:
//                if(areWeWinners && currentPose.x() > 0.8) {
//                    s_Swerve.drive(-0.5, 0, 0, true, false);
//                } else if (!areWeWinners && currentPose.x() < -0.8) {
//                    s_Swerve.drive(0.5, 0, 0, true, false);
//                } else {
                    driveController.reset();
                    phase++;
//                }
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);
                outputs = driveController.getOutputs();

                s_Intake.setSpeed(1);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

                if(isAtRoughSetpoint()) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    driveController.reset();
                    driveController.setTargetPose(pickupTarget);
                    phase++;
                }

                break;
            case 2:

                driveController.updateCurrentPose(currentPose);
                outputs = driveController.getSlowOutputs();

                s_Intake.setSpeed(1);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

                if(System.nanoTime() - timestamp > 1.5e9) {
                    s_Swerve.stop();
//                    timestamp = System.nanoTime();
//                    driveController.setTargetPose(pickupTarget);
                    isFinished = true;
                    phase++;
                }

                break;
        }
    }

    public boolean isAtSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - lineupTarget.x());
        double yError = Math.abs(s_Sparky.getPose().y() - lineupTarget.y());
        double rError = Math.abs(s_Sparky.getHeading() - lineupTarget.r());

        return xError < 0.02 && yError < 0.02 && rError < 3;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - lineupTarget.x());
        double yError = Math.abs(s_Sparky.getPose().y() - lineupTarget.y());
        double rError = Math.abs(s_Sparky.getHeading() - lineupTarget.r());

        return xError < 0.6 && yError < 0.06 && rError < 6;
    }

    public boolean isFinished() {
        return isFinished;
    }

    private void setIntakeRow() {
        if(areWeWinners) {
            switch(targetRow) {
                case 1:
                    lineupTarget = Constants.AutoConstants.RedConstants.closeBallLineup;
                    pickupTarget = Constants.AutoConstants.RedConstants.closeBallPickup;
                    break;
                case 2:
                    lineupTarget = Constants.AutoConstants.RedConstants.mediumBallLineup;
                    pickupTarget = Constants.AutoConstants.RedConstants.mediumBallPickup;
                    break;
                case 3:
                    lineupTarget = Constants.AutoConstants.RedConstants.farBallLineup;
                    pickupTarget = Constants.AutoConstants.RedConstants.farBallPickup;
                    break;
            }
        } else {
            switch(targetRow) {
                case 1:
                    lineupTarget = Constants.AutoConstants.BlueConstants.closeBallLineup;
//                    pickupTarget = Constants.AutoConstants.BlueConstants.closeBallPickup;TODO DO THIS THINGDJIOS AJDIOASJDOASOPDaskfhdsklfndsnfwk
                    break;
                case 2:
                    lineupTarget = Constants.AutoConstants.BlueConstants.mediumBallLineup;
                    break;
                case 3:
                    lineupTarget = Constants.AutoConstants.BlueConstants.farBallLineup;
                    break;
            }
        }
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
