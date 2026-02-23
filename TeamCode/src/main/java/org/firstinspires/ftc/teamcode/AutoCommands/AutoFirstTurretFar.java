package org.firstinspires.ftc.teamcode.AutoCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class AutoFirstTurretFar {

    private final Turret s_Turret;
    private final FusionOdometry s_Lemon;
    private final Limelight s_Lime;
    private double aprilBearing;
    private final double turretDefault;
    private final double turretOffset;

    private final boolean areWeFar;

    private final OmegaPose2D targetPose;

    public AutoFirstTurretFar(Turret s_Turret, FusionOdometry s_Lemon, Limelight s_Lime, boolean areWeWinners, boolean areWeFar){

        this.s_Turret = s_Turret;
        this.s_Lemon = s_Lemon;
        this.s_Lime = s_Lime;
        aprilBearing = s_Lime.getFilteredBearing();
        this.areWeFar = areWeFar;

        turretDefault = areWeWinners ? -109.92 : 109.92;

        turretOffset = areWeWinners ? 3.5 : -3.5;
        targetPose = areWeWinners ? Constants.TurretConstants.autoRedTarget : Constants.TurretConstants.autoBlueTarget;
    }

    public void execute(){

        s_Turret.setSetpoint(turretDefault);
        s_Turret.runToSetpoint();

    }
}
