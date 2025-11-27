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

public class AutoSwoopyIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final OTOSSensor s_Sparky;

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

    public AutoSwoopyIntake(Swerve s_Swerve, Intake s_Intake, Feeder s_Feeder, OTOSSensor s_Sparky, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Sparky = s_Sparky;

        finalPose = areWeWinners? Constants.AutoConstants.RedConstants.swoopyIntake : Constants.AutoConstants.BlueConstants.swoopyIntake;

//        if(areWeWinners) {
//            poses = new OmegaPose2D[]{
//                    new OmegaPose2D(
//                            Constants.AutoConstants.RedConstants.cornerPickup.x()-0.5,
//                            Constants.AutoConstants.RedConstants.cornerPickup.y()+0.05,
//                            Constants.AutoConstants.RedConstants.cornerPickup.r()),
//                    Constants.AutoConstants.RedConstants.cornerPickup
//            };
//        } else {
//            poses = new OmegaPose2D[]{
//                    new OmegaPose2D(
//                            Constants.AutoConstants.BlueConstants.cornerPickup.x()+0.5,
//                            Constants.AutoConstants.BlueConstants.cornerPickup.y()+0.05,
//                            Constants.AutoConstants.BlueConstants.cornerPickup.r()),
//                    Constants.AutoConstants.BlueConstants.cornerPickup
//            };
//        }

        driveController = new AutoDriveController();
        waypointFollower = new WaypointFollower(driveController);
    }

    public void reset(){
        isFinished = false;
        phase = 0;
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();

        s_Feeder.closeGate();
        s_Feeder.setFeederSpeed(0);
        s_Intake.setSpeed(1);

        switch(phase) {
            case 0:
//                waypointFollower.resetWaypointFollower();
                driveController.reset();
                driveController.setTargetPose(finalPose);
                phase++;
                break;
            case 1:

//                outputs = waypointFollower.getWaypointOutputs(currentPose, poses);

                driveController.updateCurrentPose(currentPose);

                outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

                if(isAtRoughSetpoint()) {
                    timestamp = System.nanoTime();
                    phase++;
                }
                break;
            case 2:
                s_Swerve.drive(0, 0.3, 0, true, false);
                if(System.nanoTime() - timestamp > 0.8e9) {
                    timestamp = System.nanoTime();
                    phase++;
                }
                break;
            case 3:
                s_Swerve.drive(0, -0.3, 0, true, false);
                if(System.nanoTime() - timestamp > 0.8e9) {
                    timestamp = System.nanoTime();
                    phase++;
                }
                break;
        }
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - finalPose.x());
        double yError = Math.abs(s_Sparky.getPose().y() - finalPose.y());
        double rError = Math.abs(s_Sparky.getHeading() - finalPose.r());

        return xError < 0.06 && yError < 0.06 && rError < 6;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
