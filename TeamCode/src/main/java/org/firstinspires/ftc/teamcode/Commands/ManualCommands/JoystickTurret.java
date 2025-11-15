package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.Controller.OmegaController;

public class JoystickTurret {

    private final Swerve s_Swerve;
    private final Turret s_Turret;

    private final OmegaController m_Operator;

    public JoystickTurret(Swerve s_Swerve, Turret s_Turret, OmegaController m_Operator){

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;

        this.m_Operator = m_Operator;
    }

    public void initialize(){

        s_Turret.setSetpoint(s_Turret.getDegrees());

    }

    public void execute(){

        double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_Operator.getLeftX(), m_Operator.getLeftY()));
        operatorJoystickAngle += 360;
        operatorJoystickAngle %= 360;

        operatorJoystickAngle -= s_Swerve.getHeading();

        operatorJoystickAngle += 360;
        operatorJoystickAngle %= 360;

        if(Math.hypot(m_Operator.getLeftX(), m_Operator.getLeftY()) > 0.9) {
            s_Turret.setSetpoint(operatorJoystickAngle);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.START)) {
            s_Turret.resetTurretPosition();
        }

        m_Operator.readButtons();

        s_Turret.runToSetpoint();
    }
}
