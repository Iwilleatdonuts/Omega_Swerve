package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoCloseShot {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FusionOdometry s_Lemon;

    private final OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;

    private boolean isFinished;

    private int phase;

    private double timestamp;

    private boolean shootSlow = false;

    public AutoCloseShot(Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake, Feeder s_Feeder, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        targetPosition = areWeWinners? Constants.NewAutoConstants.RedConstants.closeShot : Constants.NewAutoConstants.BlueConstants.closeShot;

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lemon = s_Lemon;

        driveController = new AutoDriveController();

    }

    public void reset(boolean shootSlow){
        isFinished = false;
        phase = 0;
        driveController.reset();
        this.shootSlow = shootSlow;

    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();
        double distance = s_Lemon.getDistanceFromTargetAuto(areWeWinners);
        s_Shooter.setShooterSpeed(s_Shooter.getShooterSpeedFromDistance(distance - 0.03));
        s_Shooter.setShooterAngle(s_Shooter.getShooterAngleFromDistance(distance));

        telem.putTelemetry("Phase", phase);

        telem.putTelemetry("X Target", targetPosition.x());
        telem.putTelemetry("Y Target", targetPosition.y());
        telem.putTelemetry("R Error", targetPosition.r());

        switch(phase) {
            case 0:
                s_Feeder.closeGate();
                if(areWeWinners && currentPose.x() > 1.3) {
                    s_Swerve.drive(-0.8, 0, 0, true);
                } else if (!areWeWinners && currentPose.x() < -1.3) {
                    s_Swerve.drive(0.8, 0, 0, true);
                } else {
                    driveController.reset();
                    phase++;
                }
                break;
            case 1:
                if(areWeWinners && currentPose.x() > 1.2 && shootSlow) {
                    s_Swerve.drive(-0.8, 0, 0, true);
                } else if (!areWeWinners && currentPose.x() < -1.2 && shootSlow) {
                    s_Swerve.drive(0.8, 0, 0, true);
                } else {
                    driveController.reset();
                    phase++;
                }
                break;
            case 2:

                driveController.updateCurrentPose(currentPose);
                driveController.setTargetPose(targetPosition);
                double[] outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() && s_Shooter.shooterAtSpeed()) {
                    timestamp = System.nanoTime();
                    s_Swerve.stop();
                    phase++;
                }

                break;
            case 3:
                s_Feeder.openGate();
                if(shootSlow) {
                    s_Intake.setSpeed(0.8);
                } else {
                    s_Intake.setSpeed(1);
                }
                s_Feeder.setFeederSpeed(1);
                s_Swerve.drivePrep(0, -1, 0, true);

                if((System.nanoTime() - timestamp > 1.3e9 && !shootSlow) || (System.nanoTime() - timestamp > 2e9 && shootSlow)) {
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

        return xError < 0.15 && yError < 0.15;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
