package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;

public class DriveToDashboardPoint {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;

    private SparkFunOTOS.Pose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController staticAngleController;
    private final PIDController dynamicAngleController;

    private final HolonomicDriveController holoController;

    public DriveToDashboardPoint(Swerve s_Swerve, OTOSSensor s_Sparky, EZTelemetry telem){

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;

        targetPosition = s_Swerve.getTargetPose();

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

    public void initialize(){
        xController.reset();
        yController.reset();
        staticAngleController.reset();
        dynamicAngleController.reset();
    }

    public void execute(){

        SparkFunOTOS.Pose2D currentPose = s_Sparky.getPose();
        targetPosition = new SparkFunOTOS.Pose2D(PIDTuning.randomVal0, PIDTuning.randomVal1, PIDTuning.randomVal2);

//        double rotationTarget = targetPosition.h;
//        rotationTarget = (rotationTarget + 360) % 360;
//
//        double xOutput = xController.calculate(currentPose.x, targetPosition.x);
//        double yOutput = yController.calculate(currentPose.y, targetPosition.y);
//        double rOutput = 0;
//        if(Math.abs(s_Sparky.getHeading() - rotationTarget) > 0.5) {
//            rOutput = -angleController.calculate(s_Sparky.getHeading(), rotationTarget);
//        }
//
//        s_Swerve.drive(xOutput, yOutput, rOutput, true, false);

        double[] outputs = holoController.calculate(currentPose, targetPosition);

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);

        telem.putTelemetry("X Target", targetPosition.x);
        telem.putTelemetry("Y Target", targetPosition.y);
        telem.putTelemetry("H Target", targetPosition.h);

    }

}
