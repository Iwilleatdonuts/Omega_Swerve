package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.WaypointFollower;

public class AutoCornerIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D[] targets;
    private OmegaPose2D finalPose;

    private final AutoDriveController driveController;
    private final WaypointFollower waypointFollower;

    private final boolean areWeWinners;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double[] outputs = new double[3];
    private OmegaPose2D[] poses;

    public AutoCornerIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lemon = s_Lemon;

        finalPose = areWeWinners? Constants.NewAutoConstants.RedConstants.cornerPickup : Constants.NewAutoConstants.BlueConstants.cornerPickup;

        driveController = new AutoDriveController();
        waypointFollower = new WaypointFollower(driveController);
    }

    public void reset(){
        isFinished = false;
        phase = 0;
    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();

        s_Feeder.closeGate();
        s_Feeder.setFeederSpeed(0);
        s_Intake.setSpeed(1);

        switch(phase) {
            case 0:
                driveController.reset();
                driveController.setTargetPose(finalPose);
                timestamp = System.nanoTime();
                phase++;
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);

                outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() || System.nanoTime() - timestamp > 1.5e9) {
                    timestamp = System.nanoTime();
                    phase++;
                }


                break;
            case 2:

                s_Swerve.drive(areWeWinners ? -0.7 : 0.7, 0, 0, true);

                if(System.nanoTime() - timestamp > 0.7e9) {
                    timestamp = System.nanoTime();
                    driveController.reset();
                    driveController.setTargetPose(finalPose);
                    phase++;
                }
                break;
            case 3:

                driveController.updateCurrentPose(currentPose);

                outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() || System.nanoTime() - timestamp > 1.6e9) {
                    timestamp = System.nanoTime();
                    s_Swerve.stop();
                    isFinished = true;
                    phase++;
                }
                break;
        }
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Lemon.getCurrentPose().x() - finalPose.x());
        double yError = Math.abs(s_Lemon.getCurrentPose().y() - finalPose.y());
        double rError = Math.abs(s_Lemon.getHeading() - finalPose.r());

        return xError < 0.06 && yError < 0.06 && rError < 4;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
