package org.firstinspires.ftc.teamcode.AutoCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoTurret {

    private final Turret s_Turret;
    private final FusionOdometry s_Lemon;
    private final Limelight s_Lime;
    private double aprilBearing;
    private final double turretDefault;
    private final double turretOffset;

    private final boolean areWeFar;

    private final OmegaPose2D targetPose;

    public AutoTurret(Turret s_Turret, FusionOdometry s_Lemon, Limelight s_Lime, boolean areWeWinners, boolean areWeFar){

        this.s_Turret = s_Turret;
        this.s_Lemon = s_Lemon;
        this.s_Lime = s_Lime;
        aprilBearing = s_Lime.getFilteredBearing();
        this.areWeFar = areWeFar;

        if(areWeFar) {
            turretDefault = areWeWinners? 73 : -73;
        } else {
            turretDefault = areWeWinners? 45 : -45;
        }

        turretOffset = areWeWinners ? 3.5 : -3.5;
        targetPose = areWeWinners ? Constants.TurretConstants.redTarget : Constants.TurretConstants.blueTarget;
    }

    public void execute(){

        if(s_Lime.isValidReaing()){

            aprilBearing = s_Lime.getFilteredBearing();
            double bearing = s_Turret.getDegrees() - aprilBearing;

            if(areWeFar) {
                bearing-= turretOffset;
            }

            s_Turret.setSetpoint(bearing);

        } else {

            s_Turret.setSetpoint(turretDefault);

//            OmegaPose2D currentPose = s_Lemon.getCurrentPose();
//            //gtes x and Y value on field
//
//            double theta = Math.toDegrees(Math.atan2(-(targetPose.x() - currentPose.x()), targetPose.y() - currentPose.y()));
//            double turretHeading = theta - s_Lemon.getHeading();
//
//            if(turretHeading < -180) {
//                turretHeading += 360;
//            }
//
//            if(turretHeading > 180) {
//                turretHeading -= 360;
//            }
//
//            s_Turret.setSetpoint(turretHeading);

        }
        s_Turret.runToSetpoint();

    }
}
