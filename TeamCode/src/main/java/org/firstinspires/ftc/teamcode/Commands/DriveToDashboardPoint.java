package org.firstinspires.ftc.teamcode.Commands;

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
    private final OTOSSensor s_Sparky;

    private OmegaPose2D targetPosition;

    private final AutoDriveController driveController;

    public DriveToDashboardPoint(Swerve s_Swerve, OTOSSensor s_Sparky, EZTelemetry telem){

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;

        targetPosition = s_Swerve.getTargetPose();

        driveController = new AutoDriveController();
    }

    public void initialize(){
        driveController.reset();
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();
        targetPosition = new OmegaPose2D(PIDTuning.randomVal0, PIDTuning.randomVal1, PIDTuning.randomVal2);

        driveController.setTargetPose(targetPosition);
        driveController.updateCurrentPose(currentPose);

        double[] outputs = driveController.getOutputs();

        s_Swerve.drive(outputs[0], outputs[1], outputs[2], true, false);
    }

}
