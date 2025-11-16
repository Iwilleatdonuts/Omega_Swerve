package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.Pose2D;

public class AutoCloseShot {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final OTOSSensor s_Sparky;

    private Pose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController staticAngleController;
    private final PIDController dynamicAngleController;

    private final HolonomicDriveController holoController;

    private final boolean areWeWinners;

    private boolean isFinished;

    private int phase;

    private double timestamp;

    public AutoCloseShot(Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake, Feeder s_Feeder, OTOSSensor s_Sparky, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.telem = telem;

        targetPosition = areWeWinners? Constants.AutoConstants.RedConstants.closeShot : Constants.AutoConstants.BlueConstants.closeShot;

        this.s_Swerve = s_Swerve;
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Sparky = s_Sparky;

        xController = new PIDController(2.2, 0, 0.05);
        yController = new PIDController(2.2, 0, 0.05);

        staticAngleController = new PIDController(0.006, 0.02, 0.00015);
        staticAngleController.setIZone(40);
        staticAngleController.enableContinuousInput(0, 360);

        dynamicAngleController = new PIDController(0.0025, 0.015, 0.0001);
        dynamicAngleController.setIZone(40);
        dynamicAngleController.enableContinuousInput(0, 360);

        holoController = new HolonomicDriveController(xController, yController, staticAngleController, dynamicAngleController);
    }

    public void reset(){
        isFinished = false;
        phase = 0;
        xController.reset();
        yController.reset();
        staticAngleController.reset();
        dynamicAngleController.reset();
    }

    public void execute(){

        Pose2D currentPose = s_Sparky.getPose();
        double currentHeading = s_Sparky.getHeading();
        s_Shooter.setShooterSpeed(0.36);

        telem.putTelemetry("Phase", phase);

        switch(phase) {
            case 0:
                s_Feeder.closeGate();
                if(areWeWinners && currentPose.x() > 1.21) {
                    s_Swerve.drive(-0.8, 0, 0, true, false);
                } else if (!areWeWinners && currentPose.x() < -1.21) {
                    s_Swerve.drive(0.8, 0, 0, true, false);
                } else {
                    xController.reset();
                    yController.reset();
                    staticAngleController.reset();
                    dynamicAngleController.reset();
                    phase++;
                }
                break;
            case 1:

                double[] outputs = holoController.calculate(currentPose, targetPosition);

//                double headingError = Math.abs(targetPosition.r() - currentPose.r());
//
//                double rOutput = 0;
//
//                if(headingError < 5) {
//                    rOutput = -dynamicAngleController.calculate(currentPose.r(), targetPosition.r());
//                    staticAngleController.reset();
//                } else {
//                    rOutput = -staticAngleController.calculate(currentHeading, targetPosition.r());
//                    dynamicAngleController.reset();
//                }
                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);
//                telem.putTelemetry("R Setpoint", targetPosition.r());
//                telem.putTelemetry("R pose}", currentHeading);

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

                if(System.nanoTime() - timestamp > 2e9) {
                    isFinished = true;
                }

                break;
        }
    }

    public boolean isAtSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - targetPosition.x());
        double yError = Math.abs(s_Sparky.getPose().y() - targetPosition.y());
        double rError = Math.abs(s_Sparky.getHeading() - targetPosition.r());

        return xError < 0.02 && yError < 0.02 && rError < 3;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - targetPosition.x());
        double yError = Math.abs(s_Sparky.getPose().y() - targetPosition.y());
        double rError = Math.abs(s_Sparky.getHeading() - targetPosition.r());

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
