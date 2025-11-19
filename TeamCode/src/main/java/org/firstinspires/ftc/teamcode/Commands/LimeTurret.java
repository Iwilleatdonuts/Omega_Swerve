package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.Controller.OmegaController;

public class LimeTurret {

    private final Swerve s_Swerve;
    private final Turret s_Turret;
    private final Limelight s_Lime;
    private double aprilBearing;

    private final OmegaController m_Operator;

    public LimeTurret(Swerve s_Swerve, Turret s_Turret, Limelight s_Lime, OmegaController m_Operator){

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;

        this.m_Operator = m_Operator;
    }

    public void initialize(){

        aprilBearing = s_Lime.getGoalBearing();

    }

    public void execute(){


        if(s_Lime.isValidReaing()){

            aprilBearing = s_Lime.getGoalBearing();
            double bearing = s_Turret.getDegrees() - aprilBearing;

            s_Turret.setSetpoint(bearing);

        } else {

            double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_Operator.getLeftX(), m_Operator.getLeftY()));
            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            operatorJoystickAngle -= s_Swerve.getHeading();

            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            if(Math.hypot(m_Operator.getLeftX(), m_Operator.getLeftY()) > 0.9) {
                s_Turret.setSetpoint(operatorJoystickAngle);
            }

        }

        s_Turret.runToSetpoint();
    }

    public void end() {
        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Turret.setSpeed(0);
    }
}
