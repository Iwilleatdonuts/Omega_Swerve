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

public class AutoSwoopyIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D[] targets;
    private OmegaPose2D firstPost;
    private OmegaPose2D finalPose;

    private final AutoDriveController driveController;
    private final WaypointFollower waypointFollower;

    private final boolean areWeWinners;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double[] outputs = new double[3];
    private OmegaPose2D[] poses;

    public AutoSwoopyIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lemon = s_Lemon;

        firstPost = areWeWinners? Constants.NewAutoConstants.RedConstants.cornerPickup : Constants.NewAutoConstants.BlueConstants.cornerPickup;
        finalPose = new OmegaPose2D(firstPost.x(), firstPost.y()+0.5, firstPost.r());

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
//                waypointFollower.resetWaypointFollower();
                driveController.reset();
                driveController.setTargetPose(firstPost);
                timestamp = System.nanoTime();
                phase++;
                break;
            case 1:
                driveController.updateCurrentPose(currentPose);

                outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() || System.nanoTime() - timestamp > 2e9) {
                    timestamp = System.nanoTime();
                    phase++;
                }
                break;
            case 2:
                s_Swerve.drive(areWeWinners ? -1 : 1, 0.3, 0, true);
                if(System.nanoTime() - timestamp > 0.4e9) {
                    timestamp = System.nanoTime();
                    driveController.setTargetPose(finalPose);
                    phase++;
                }
                break;
            case 3:
                driveController.updateCurrentPose(currentPose);

                outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() || System.nanoTime() - timestamp > 2e9) {
                    timestamp = System.nanoTime();
                    isFinished = true;
                    phase++;
                }
                break;
        }
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Lemon.getCurrentPose().x() - firstPost.x());
        double yError = Math.abs(s_Lemon.getCurrentPose().y() - firstPost.y());
        double rError = Math.abs(s_Lemon.getHeading() - firstPost.r());

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
