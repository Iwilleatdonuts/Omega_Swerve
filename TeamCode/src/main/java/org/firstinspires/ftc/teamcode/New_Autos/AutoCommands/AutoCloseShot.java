package org.firstinspires.ftc.teamcode.New_Autos.AutoCommands;

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

    private final Swerve swerve;
    private final Shooter shooter;
    private final Intake intake;
    private final Feeder feeder;
    private final FusionOdometry lemon;

    private final AutoDriveController driveController;

    private OmegaPose2D targetPosition;

    private final boolean areWeWinners;
    private boolean isFinished;

    private int phase;
    private double timestamp;

    public AutoCloseShot(EZTelemetry telem, Swerve swerve, Shooter shooter, Intake intake, Feeder feeder, FusionOdometry lemon, boolean areWeWinners){

        this.telem = telem;
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.feeder = feeder;
        this.lemon = lemon;

        this.areWeWinners = areWeWinners;

        targetPosition = areWeWinners ? Constants.NewAutoConstants.RedConstants.closeShot : Constants.NewAutoConstants.BlueConstants.closeShot;

        driveController = new AutoDriveController();

    }

    public void reset() {
        isFinished = false;
        phase = 0;
        driveController.reset();
    }

    public void execute() {

        OmegaPose2D currentPose = lemon.getCurrentPose();
        double distance = Math.hypot(lemon.getCurrentPose().x() - targetPosition.x(), lemon.getCurrentPose().y() - targetPosition.y());
        shooter.setShooterSpeed(shooter.getShooterSpeedFromDistance(distance));
        shooter.setShooterAngle(shooter.getShooterAngleFromDistance(distance));

        switch(phase) {
            case 0:
                feeder.closeGate();
                if(areWeWinners && currentPose.x() > 1.4) {
                    swerve.drive(-0.8, 0, 0, true);
                } else if (!areWeWinners && currentPose.x() < -1.4) {
                    swerve.drive(0.8, 0, 0, true);
                } else {
                    driveController.reset();
                    phase++;
                }
                break;
            case 1:
                driveController.updateCurrentPose(currentPose);
                driveController.setTargetPose(targetPosition);
                double[] outputs = driveController.getOutputs();

                swerve.drive(outputs[0], outputs[1], outputs[2], true);

        }

    }

    public boolean isAtSetpoint() {
        double xError = Math.abs(lemon.getCurrentPose().x() - targetPosition.x());
        double yError = Math.abs(lemon.getCurrentPose().y() - targetPosition.y());
        double rError = Math.abs(lemon.getHeading() - targetPosition.r());

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
