package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.Pose2D;

public class DriveToAutoPoint {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;

    private Pose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController staticAngleController;
    private final PIDController dynamicAngleController;

    private final HolonomicDriveController holoController;

    public DriveToAutoPoint(Swerve s_Swerve, OTOSSensor s_Sparky, EZTelemetry telem){

        this.telem = telem;

        targetPosition = new Pose2D(s_Sparky.getPose().x(), s_Sparky.getPose().y(), s_Sparky.getHeading());

        this.s_Swerve = s_Swerve;
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
//        angleController = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        dynamicAngleController.setIZone(40);
        dynamicAngleController.enableContinuousInput(0, 360);

        holoController = new HolonomicDriveController(xController, yController, staticAngleController, dynamicAngleController);
    }

    public void initialize(Pose2D pose){
        xController.reset();
        yController.reset();
        staticAngleController.reset();
        dynamicAngleController.reset();
        targetPosition = pose;
    }

    public void execute(){

        Pose2D currentPose = s_Sparky.getPose();

        double[] outputs = holoController.calculate(currentPose, targetPosition);

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

        telem.putTelemetry("X Target", targetPosition.x());
        telem.putTelemetry("Y Target", targetPosition.y());
        telem.putTelemetry("H Target", targetPosition.r());

    }

    public boolean isAtSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - targetPosition.x());
        double yError = Math.abs(s_Sparky.getPose().y() - targetPosition.y());
        double rError = Math.abs(s_Sparky.getHeading() - targetPosition.r());

        return xError < 0.02 && yError < 0.02 && rError < 0.5;
    }

    public boolean isAtRoughSetpoint(){
        double xError = Math.abs(s_Sparky.getPose().x() - targetPosition.x());
        double yError = Math.abs(s_Sparky.getPose().y() - targetPosition.y());
        double rError = Math.abs(s_Sparky.getHeading() - targetPosition.r());

        return xError < 0.1 && yError < 0.1 && rError < 10;
    }

}
