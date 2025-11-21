package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
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

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rController;

    private final HolonomicDriveController holoController;

    private double rGoalPrev;

    public DriveToDashboardPoint(Swerve s_Swerve, OTOSSensor s_Sparky, EZTelemetry telem){

        this.telem = telem;

        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;

        targetPosition = s_Swerve.getTargetPose();

        xController = new PIDController(2.2, 0, 0.05);
        yController = new PIDController(2.2, 0, 0.05);
        xController.setIZone(0.5);
        yController.setIZone(0.5);

        rController = new ProfiledPIDController(0.006, 0.02, 0.00015, new TrapezoidProfile.Constraints(1.93, 3));
        rController.setIZone(40);
        rController.enableContinuousInput(0, 360);

        holoController = new HolonomicDriveController(xController, yController, rController);

        rGoalPrev = s_Sparky.getHeading();
    }

    public void initialize(){
        xController.reset();
        yController.reset();
        rController.reset(s_Sparky.getHeading());
    }

    public void execute(){

        OmegaPose2D currentPose = s_Sparky.getPose();
        targetPosition = new OmegaPose2D(PIDTuning.randomVal0, PIDTuning.randomVal1, PIDTuning.randomVal2);

        if(targetPosition.r() != rGoalPrev) {
            rController.reset(s_Sparky.getHeading());
        }

        ChassisSpeeds speeds = holoController.calculate(OmegaPose2D.OmegaPoseToWPIPose(currentPose), OmegaPose2D.OmegaPoseToWPIPose(targetPosition), 1.2, Rotation2d.fromDegrees(targetPosition.r()));

        s_Swerve.drive(speeds);
        rGoalPrev = targetPosition.r();
    }

}
