package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoLeaveFarZone {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;

    private boolean isFinished;

    private int phase;

    private double timestamp;

    public AutoLeaveFarZone(Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake, Feeder s_Feeder, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        targetPosition = areWeWinners? Constants.AutoConstants.RedConstants.farBallLineup : Constants.AutoConstants.BlueConstants.farBallLineup;

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lemon = s_Lemon;

        driveController = new AutoDriveController();

    }

    public void reset(){
        isFinished = false;
        phase = 0;
        driveController.reset();
    }

    public void execute(){

        s_Shooter.setShooterSpeed(0);
        s_Intake.setSpeed(0);
        s_Feeder.setFeederSpeed(0);

        switch(phase) {

            case 0:
                driveController.reset();
                phase++;
                break;
            case 1:

                driveController.updateCurrentPose(s_Lemon.getCurrentPose());
                driveController.setTargetPose(targetPosition);
                double[] outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

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

        return xError < 0.1 && yError < 0.1 && rError < 10;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
