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

    private final PIDController translationController;
    private final PIDController angleController;

    public DriveToPoint(Swerve s_Swerve, OTOSSensor s_Sparky, SparkFunOTOS.Pose2D targetPosition){

        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;

        this.targetPosition = targetPosition;

        translationController = new PIDController(0.002, 0.0005, 0);

        angleController = new PIDController(0.0026, 0.01, 0);
        angleController.enableContinuousInput(0, 360);

        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){

        SparkFunOTOS.Pose2D currentPose = s_Sparky.getPose();

        double rotationTarget = targetPosition.h;
        rotationTarget = (rotationTarget + 360) % 360;

        double xOutput = translationController.calculate(currentPose.x, targetPosition.x);
        double yOutput = translationController.calculate(currentPose.y, targetPosition.y);
        double rOutput = -angleController.calculate(s_Sparky.getHeading(), rotationTarget);

        s_Swerve.drive(xOutput, yOutput, rOutput, true, false);

    }

}
