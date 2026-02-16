package org.firstinspires.ftc.teamcode.New_Autos.AutoCommands;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AAutoCommandTemplate {

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

    public AAutoCommandTemplate(EZTelemetry telem, Swerve swerve, Shooter shooter, Intake intake, Feeder feeder, FusionOdometry lemon, boolean areWeWinners){

        this.telem = telem;
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.feeder = feeder;
        this.lemon = lemon;

        this.areWeWinners = areWeWinners;

        driveController = new AutoDriveController();

    }

    public void reset() {
        isFinished = false;
        phase = 0;
        driveController.reset();
    }

    public void execute() {

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
