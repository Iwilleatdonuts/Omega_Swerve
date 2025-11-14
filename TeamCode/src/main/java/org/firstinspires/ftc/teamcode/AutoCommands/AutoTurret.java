package org.firstinspires.ftc.teamcode.AutoCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class AutoTurret {

    private final Turret s_Turret;
    private final AprilVisionOnTurret s_Vision;
    private double aprilBearing;

    public AutoTurret(Turret s_Turret, AprilVisionOnTurret s_Vision){

        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;
        aprilBearing = s_Vision.getAdjustedGoalBearing();

    }

    public void execute(){

        if(s_Vision.hasGoalTag()){

            aprilBearing = s_Vision.getAdjustedGoalBearing();
            double bearing = s_Turret.getDegrees() + aprilBearing;

            s_Turret.setSetpoint(bearing);

        } else {

            s_Turret.setSetpoint(45);

        }
        s_Turret.runToSetpoint();

    }
}
