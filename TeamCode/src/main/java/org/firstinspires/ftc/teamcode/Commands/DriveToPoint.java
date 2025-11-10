package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;

public class DriveToPoint extends CommandBase {

    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;

    private SparkFunOTOS.Pose2D targetPosition;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController angleController;

    public DriveToPoint(Swerve s_Swerve, OTOSSensor s_Sparky){

        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;

        targetPosition = s_Swerve.getTargetPose();

        xController = new PIDController(2.2, 0, 0.05);
        yController = new PIDController(2.2, 0, 0.05);
//        xController = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);
//        yController = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);
        xController.setIZone(PIDTuning.k1IZone);
        yController.setIZone(PIDTuning.k1IZone);

        angleController = new PIDController(0.006, 0.02, 0.00015);
//        angleController = new PIDController(PIDTuning.k2P, PIDTuning.k2I, PIDTuning.k2D);
        angleController.setIZone(40);
        angleController.enableContinuousInput(0, 360);

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
        xController.reset();
        yController.reset();
        angleController.reset();
    }

    @Override
    public void execute(){

        SparkFunOTOS.Pose2D currentPose = s_Sparky.getPose();
        targetPosition = s_Swerve.getTargetPose();

        double rotationTarget = targetPosition.h;
        rotationTarget = (rotationTarget + 360) % 360;

        double xOutput = xController.calculate(currentPose.x, targetPosition.x);
        double yOutput = yController.calculate(currentPose.y, targetPosition.y);
        double rOutput = 0;
        if(Math.abs(s_Sparky.getHeading() - rotationTarget) > 0.5) {
            rOutput = -angleController.calculate(s_Sparky.getHeading(), rotationTarget);
        }

        s_Swerve.drive(xOutput, yOutput, rOutput, true, false);

    }

}
