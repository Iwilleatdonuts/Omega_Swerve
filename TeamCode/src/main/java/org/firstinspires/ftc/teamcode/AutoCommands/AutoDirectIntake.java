package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoDirectIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FusionOdometry s_Lemion;

    private OmegaPose2D lineupTarget;
    private OmegaPose2D pickupTarget;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;
    private int targetRow;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double[] outputs = new double[3];

    public AutoDirectIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners, int targetRow){

        this.targetRow = targetRow;

        this.areWeWinners = areWeWinners;

        this.telem = telem;

       setIntakeRow();

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lemion = s_Lemon;

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

        OmegaPose2D currentPose = s_Lemion.getCurrentPose();

        s_Feeder.closeGate();
        s_Feeder.setFeederSpeed(0);

        switch(phase) {
            case 0:
                    driveController.reset();
                    timestamp = System.nanoTime();
                    phase++;
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);
                outputs = driveController.getOutputs();

                s_Intake.setSpeed(1);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() || System.nanoTime() - timestamp > 3.5e9) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    driveController.reset();
                    driveController.setTargetPose(pickupTarget);
                    phase++;
                }

                break;
            case 2:

                driveController.updateCurrentPose(currentPose);
                outputs = driveController.getOutputs();

                s_Intake.setSpeed(1);

                s_Swerve.drivePrep(outputs[0], outputs[1], outputs[2], true);

                if(System.nanoTime() - timestamp > 0.2) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    driveController.reset();
                    driveController.setTargetPose(pickupTarget);
                    phase++;
                }

                break;
            case 3:

                driveController.updateCurrentPose(currentPose);
                outputs = driveController.getOutputs();

                s_Intake.setSpeed(1);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(System.nanoTime() - timestamp > 1.5e9 || s_Intake.hasThreeBalls()) {
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
        double xError = Math.abs(s_Lemion.getCurrentPose().x() - lineupTarget.x());
        double yError = Math.abs(s_Lemion.getCurrentPose().y() - lineupTarget.y());
        double rError = Math.abs(s_Lemion.getHeading() - lineupTarget.r());

        return xError < 0.02 && yError < 0.02 && rError < 3;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Lemion.getCurrentPose().x() - lineupTarget.x());
        double yError = Math.abs(s_Lemion.getCurrentPose().y() - lineupTarget.y());
        double rError = Math.abs(s_Lemion.getHeading() - lineupTarget.r());

        return xError < 0.06 && yError < 0.06 && rError < 4;
    }

    public boolean isFinished() {
        return isFinished;
    }

    private void setIntakeRow() {
        if(areWeWinners) {
            switch(targetRow) {
                case 1:
                    lineupTarget = Constants.NewAutoConstants.RedConstants.firstSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.RedConstants.firstSpikePickup;
                    break;
                case 2:
                    lineupTarget = Constants.NewAutoConstants.RedConstants.secondSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.RedConstants.secondSpikePickup;
                    break;
                case 3:
                    lineupTarget = Constants.NewAutoConstants.RedConstants.thirdSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.RedConstants.thirdSpikePickup;
                    break;
            }
        } else {
            switch(targetRow) {
                case 1:
                    lineupTarget = Constants.NewAutoConstants.BlueConstants.firstSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.BlueConstants.firstSpikePickup;
                    break;
                case 2:
                    lineupTarget = Constants.NewAutoConstants.BlueConstants.secondSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.BlueConstants.secondSpikePickup;
                    break;
                case 3:
                    lineupTarget = Constants.NewAutoConstants.BlueConstants.thirdSpikeLineup;
                    pickupTarget = Constants.NewAutoConstants.BlueConstants.thirdSpikePickup;
                    break;
            }
        }
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
