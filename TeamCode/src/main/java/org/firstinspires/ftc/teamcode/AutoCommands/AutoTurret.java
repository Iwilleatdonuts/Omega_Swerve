package org.firstinspires.ftc.teamcode.AutoCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class AutoTurret {

    private final Turret s_Turret;
    private final Limelight s_Lime;
    private double aprilBearing;
    private final double turretDefault;
    private final double turretOffset;

    private final boolean areWeFar;

    public AutoTurret(Turret s_Turret, Limelight s_Lime, boolean areWeWinners, boolean areWeFar){

        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;
        aprilBearing = s_Lime.getFilteredBearing();
        this.areWeFar = areWeFar;

        if(areWeFar) {
            turretDefault = areWeWinners? 70 : -70;
        } else {
            turretDefault = areWeWinners? 45 : -45;
        }

        turretOffset = areWeWinners ? 3.5 : -3.5;
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

        }
        s_Turret.runToSetpoint();

    }
}
