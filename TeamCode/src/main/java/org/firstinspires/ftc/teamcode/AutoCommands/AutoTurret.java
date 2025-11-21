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

    public AutoTurret(Turret s_Turret, Limelight s_Lime){

        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;
        aprilBearing = s_Lime.getFilteredBearing();

    }

    public void execute(){

        if(s_Lime.isValidReaing()){

            aprilBearing = s_Lime.getFilteredBearing();
            double bearing = s_Turret.getDegrees() - aprilBearing;

            s_Turret.setSetpoint(bearing);

        } else {

            s_Turret.setSetpoint(45);

        }
        s_Turret.runToSetpoint();

    }
}
