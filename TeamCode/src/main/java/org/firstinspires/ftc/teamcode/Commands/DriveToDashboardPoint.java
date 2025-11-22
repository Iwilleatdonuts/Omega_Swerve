package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.Utilities.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utilities.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utilities.math.trajectory.TrapezoidProfile;

public class DriveToDashboardPoint {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final OTOSSensor s_Sparky;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    public DriveToDashboardPoint(Swerve s_Swerve, Intake s_Intake, OTOSSensor s_Sparky, EZTelemetry telem){

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Sparky = s_Sparky;

        targetPosition = s_Swerve.getTargetPose();

        driveController = new AutoDriveController();
    }

    public void initialize(){
        driveController.reset();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();
        int foo = (int)PIDTuning.randomVal0;
        switch(foo) {
            case 0:
                targetPosition = new OmegaPose2D(0, 0, 0);
                break;
            case 1:
                targetPosition = Constants.AutoConstants.RedConstants.closeShot;
                break;
            case 2:
                targetPosition = Constants.AutoConstants.RedConstants.closeBallLineup;
                break;
            case 3:
                targetPosition = Constants.AutoConstants.RedConstants.closeBallPickup;
                break;
            case 4:
                targetPosition = Constants.AutoConstants.RedConstants.mediumBallLineup;
                break;
            case 5:
                targetPosition = Constants.AutoConstants.RedConstants.mediumBallPickup;
                break;
            case 6:
                targetPosition = Constants.AutoConstants.RedConstants.farBallLineup;
                break;
            case 7:
                targetPosition = Constants.AutoConstants.RedConstants.farBallPickup;
                break;
            case 8:
                targetPosition = Constants.AutoConstants.RedConstants.gateLineup;
                break;
            case 9:
                targetPosition = Constants.AutoConstants.RedConstants.gatePush;
                break;
            case 10:
                targetPosition = Constants.AutoConstants.RedConstants.closeStart;
                break;
        }

        if(PIDTuning.randomVal1 == 1) {
            s_Intake.setSpeed(1);
        } else if (PIDTuning.randomVal1 == 2) {
            s_Intake.setSpeed(-1);
        } else {
            s_Intake.setSpeed(0);
        }

        driveController.setTargetPose(targetPosition);
        driveController.updateCurrentPose(currentPose);

        double[] outputs;
        double[] xOutputs;

        if(foo == 3 || foo == 5 || foo == 7 || foo == 8 || foo == 9) {
            outputs = driveController.getSlowOutputs();
//            xOutputs = driveController.getSlowOutputs();
//            outputs = new double[]{xOutputs[0], 0, 0};
        } else {
            outputs = driveController.getOutputs();
        }

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);
    }

}
