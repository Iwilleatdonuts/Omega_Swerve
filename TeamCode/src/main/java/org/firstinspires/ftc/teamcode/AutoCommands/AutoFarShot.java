package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoFarShot {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Turret s_Turret;
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

    public AutoFarShot(Swerve s_Swerve, Shooter s_Shooter, Turret s_Turret, Intake s_Intake, Feeder s_Feeder, Limelight s_Lime, FusionOdometry s_Lemno, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        targetPosition = areWeWinners? Constants.AutoConstants.RedConstants.farShot : Constants.AutoConstants.BlueConstants.farShot;

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Turret = s_Turret;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Lime = s_Lime;
        this.s_Lemon = s_Lemno;

        driveController = new AutoDriveController();

    }

    public void reset(){
        shooterSpeed = 0.58;
        isFinished = false;
        phase = 0;
        driveController.reset();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();
        double distance = s_Lime.getFilteredDistance();

        if(distance != 0) {
            shooterSpeed = s_Shooter.getShooterSpeedFromDistance(distance);
        }

        s_Shooter.setShooterSpeed(shooterSpeed);
        s_Shooter.setShooterAngle(Constants.ShooterConstants.farAngle);

        telem.putTelemetry("Phase", phase);

        switch(phase) {
            case 0:
                s_Feeder.closeGate();
                driveController.reset();
                s_Intake.setSpeed(1);
                phase++;
                break;
            case 1:

                driveController.updateCurrentPose(currentPose);
                driveController.setTargetPose(targetPosition);
                double[] outputs = driveController.getOutputs();

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);
                s_Intake.setSpeed(1);

                if(isAtRoughSetpoint() && s_Shooter.shooterAtSpeed() && s_Turret.atRoughSetpoint()) {
                    timestamp = System.nanoTime();
                    s_Swerve.stop();
                    phase++;
                }

                break;
            case 2:

                s_Feeder.openGate();
                s_Feeder.setFeederSpeed(1);
                s_Intake.setSpeed(0.8);

                if(System.nanoTime() - timestamp > 2e9) {
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
