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

public class AutoMediumShot {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final Limelight s_Lime;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    private final boolean areWeWinners;

    private boolean isFinished;

    private int phase;

    private double timestamp;
    private double shooterSpeed;
    private double shooterAngle;

    public AutoMediumShot(Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake, Feeder s_Feeder, Limelight s_Lime, FusionOdometry s_Lemon, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        targetPosition = areWeWinners? Constants.NewAutoConstants.RedConstants.finalCloseShot : Constants.NewAutoConstants.BlueConstants.finalCloseShot;

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lime = s_Lime;
        this.s_Lemon = s_Lemon;

        driveController = new AutoDriveController();

    }

    public void reset(){
        shooterSpeed = 0.42;
        shooterAngle = 0.7;
        isFinished = false;
        phase = 0;
        driveController.reset();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();
        double distance = s_Lime.getFilteredDistance();

        if(distance != 0) {
            shooterSpeed = s_Shooter.getShooterSpeedFromDistance(distance);
            shooterAngle = s_Shooter.getShooterAngleFromDistance(distance);
        }
        s_Shooter.setShooterSpeed(shooterSpeed);
        s_Shooter.setShooterAngle(shooterAngle);

        telem.putTelemetry("Phase", phase);

        switch(phase) {
            case 0:
                s_Feeder.closeGate();
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
                driveController.setTargetPose(targetPosition);
                double[] outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);

                if(isAtRoughSetpoint() && s_Shooter.shooterAtSpeed()) {
                    timestamp = System.nanoTime();
                    s_Swerve.stop();
                    phase++;
                }

                break;
            case 2:

                s_Feeder.openGate();
                s_Feeder.setFeederSpeed(1);
                s_Intake.setSpeed(1);

                if(System.nanoTime() - timestamp > 1.7e9) {
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
