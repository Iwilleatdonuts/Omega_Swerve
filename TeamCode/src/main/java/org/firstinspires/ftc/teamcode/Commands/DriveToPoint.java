package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;

public class DriveToPoint {

    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;

    private OmegaPose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController staticAngleController;
    private final PIDController dynamicAngleController;

    public DriveToPoint(Swerve s_Swerve, OTOSSensor s_Sparky){

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

        dynamicAngleController = new PIDController(0.006, 0.02, 0.00015);
//        angleController = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        dynamicAngleController.setIZone(40);
        dynamicAngleController.enableContinuousInput(0, 360);

    }

    public void initialize(){
        xController.reset();
        yController.reset();
        staticAngleController.reset();
        dynamicAngleController.reset();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();
        targetPosition = s_Swerve.getTargetPose();

        double rotationTarget = targetPosition.r();

        double xOutput = xController.calculate(currentPose.x(), targetPosition.x());
        double yOutput = yController.calculate(currentPose.y(), targetPosition.y());
        double rOutput = 0;
        if(Math.abs(s_Sparky.getHeading() - rotationTarget) > 0.5) {
            if(xOutput != 0 || yOutput != 0) {
                rOutput = -dynamicAngleController.calculate(s_Sparky.getHeading(), rotationTarget);
            } else {
                rOutput = -staticAngleController.calculate(s_Sparky.getHeading(), rotationTarget);
            }
        }

        s_Swerve.drive(xOutput, yOutput, rOutput, true, false);

    }

}
