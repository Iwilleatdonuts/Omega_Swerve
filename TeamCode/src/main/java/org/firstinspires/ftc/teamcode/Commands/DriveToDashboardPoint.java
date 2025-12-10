package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.DriveTuner;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class DriveToDashboardPoint {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D targetPosition;

    private AutoDriveController driveController;

    private final OmegaController controller;
    public DriveToDashboardPoint(Swerve s_Swerve, Intake s_Intake, FusionOdometry s_Lemon, EZTelemetry telem, OmegaController controller){

        this.controller = controller;

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Lemon = s_Lemon;

        driveController = new AutoDriveController();
    }

    public void initialize(){
        driveController.reset();
    }

    public void execute(){

        controller.readButtons();

        if(controller.wasJustPressed(GamepadKeys.Button.A)) {
            driveController.initializeControllers();
        }

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();
        int foo = (int) DriveTuner.targetPoseIndex;
        switch(foo) {
//            case 0:
//                targetPosition = new OmegaPose2D(0, 0, 0);
//                break;
//            case 1:
//                targetPosition = new OmegaPose2D(-1.5, 0, 0);
//                break;
//            case 2:
//                targetPosition = new OmegaPose2D(-1.5, 1, 0);
//                break;
//            case 3:
//                targetPosition = new OmegaPose2D(0, 1, 0);
//                break;
//            case 4:
//                targetPosition = new OmegaPose2D(-0.5, 0.5, 270);
//                break;
//            case 5:
//                targetPosition = new OmegaPose2D(-1, 0.5, 90);
//                break;

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

        if(DriveTuner.runIntakeIndex == 1) {
            s_Intake.setSpeed(1);
        } else if (DriveTuner.runIntakeIndex == 2) {
            s_Intake.setSpeed(-1);
        } else {
            s_Intake.setSpeed(0);
        }

        driveController.setTargetPose(targetPosition);
        driveController.updateCurrentPose(currentPose);

        double[] outputs;

//        if(foo == 3 || foo == 5 || foo == 7 || foo == 8 || foo == 9) {
//            outputs = driveController.getSlowOutputs();
//        } else {
            outputs = driveController.getOutputs();
//        }

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);
    }

}
