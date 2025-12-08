package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.AutoDriveController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;
import org.firstinspires.ftc.teamcode.Utilities.WaypointFollower;

public class DriveToDashboardPointWithSwerb {

    private final EZTelemetry telem;

    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final FusionOdometry s_Lemon;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;
    private final WaypointFollower waypointFollower;

    public DriveToDashboardPointWithSwerb(Swerve s_Swerve, Intake s_Intake, FusionOdometry s_Lemon, EZTelemetry telem){

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.s_Lemon = s_Lemon;

//        targetPosition = s_Swerve.getTargetPose();

        driveController = new AutoDriveController();
        waypointFollower = new WaypointFollower(driveController);
    }

    public void initialize(){
        driveController.reset();
        waypointFollower.resetWaypointFollower();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Lemon.getCurrentPose();
        int foo = (int)PIDTuning.randomVal0;

        if(PIDTuning.randomVal1 == 1) {
            s_Intake.setSpeed(1);
        } else if (PIDTuning.randomVal1 == 2) {
            s_Intake.setSpeed(-1);
        } else {
            s_Intake.setSpeed(0);
        }

        OmegaPose2D[] poses = {
                new OmegaPose2D(0, 0, 0),
                new OmegaPose2D(0, 1, 0),
                new OmegaPose2D(1, 2, 90),
                new OmegaPose2D(1, 1, 180),
                new OmegaPose2D(0, 0, 0)
        };

        double[] outputs = waypointFollower.getWaypointOutputs(currentPose, poses);

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true);
    }

}
