package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.Pose2D;

public class AutoIntake {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final OTOSSensor s_Sparky;

    private Pose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController staticAngleController;
    private final PIDController dynamicAngleController;

    private final HolonomicDriveController holoController;

    private final boolean areWeWinners;
    private int targetRow;

    private int phase;

    private double timestamp;

    private boolean isFinished;

    public AutoIntake(Swerve s_Swerve, Intake s_Intake, OTOSSensor s_Sparky, EZTelemetry telem, boolean areWeWinners, int targetRow){

        this.targetRow = targetRow;

        this.areWeWinners = areWeWinners;

        this.telem = telem;

       setIntakeRow();

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Sparky = s_Sparky;

        xController = new PIDController(2.2, 0, 0.05);
        yController = new PIDController(2.2, 0, 0.05);
//        xController = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);
//        yController = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);

        staticAngleController = new PIDController(0.006, 0.02, 0.00015);
//        angleController = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        staticAngleController.setIZone(40);
        staticAngleController.enableContinuousInput(0, 360);

        dynamicAngleController = new PIDController(0.0025, 0.015, 0.0001);
//        dynamicAngleController = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        dynamicAngleController.setIZone(40);
        dynamicAngleController.enableContinuousInput(0, 360);

        holoController = new HolonomicDriveController(xController, yController, staticAngleController, dynamicAngleController);
    }

    public void reset(int newRow){
        isFinished = false;
        phase = 0;
        targetRow = newRow;
        setIntakeRow();
        xController.reset();
        yController.reset();
        staticAngleController.reset();
        dynamicAngleController.reset();
    }

    public void execute(){

        Pose2D currentPose = s_Sparky.getPose();

        switch(phase) {
            case 0:
                if(areWeWinners && currentPose.x() > 0.6) {
                    s_Swerve.drive(-0.8, 0, 0, true, false);
                } else if (!areWeWinners && currentPose.x() < -0.6) {
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

                s_Intake.setSpeed(1);

                s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

                if(isAtRoughSetpoint()) {
                    s_Swerve.stop();
                    timestamp = System.nanoTime();
                    phase++;
                }

                break;
            case 2:

                s_Intake.setSpeed(1);

                s_Swerve.drive(0.2, 0, 0, true, false);

                if(System.nanoTime() - timestamp > 2.5e9) {
                    s_Swerve.stop();
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

        return xError < 0.6 && yError < 0.06 && rError < 6;
    }

    public boolean isFinished() {
        return isFinished;
    }

    private void setIntakeRow() {
        if(areWeWinners) {
            switch(targetRow) {
                case 1:
                    targetPosition = Constants.AutoConstants.RedConstants.closeBallLineup;
                    break;
                case 2:
                    targetPosition = Constants.AutoConstants.RedConstants.mediumBallLineup;
                    break;
                case 3:
                    targetPosition = Constants.AutoConstants.RedConstants.farBallLineup;
                    break;
            }
        } else {
            switch(targetRow) {
                case 1:
                    targetPosition = Constants.AutoConstants.BlueConstants.closeBallLineup;
                    break;
                case 2:
                    targetPosition = Constants.AutoConstants.BlueConstants.mediumBallLineup;
                    break;
                case 3:
                    targetPosition = Constants.AutoConstants.BlueConstants.farBallLineup;
                    break;
            }
        }
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
