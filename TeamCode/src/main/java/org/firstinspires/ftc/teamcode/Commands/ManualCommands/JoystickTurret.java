package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class JoystickTurret extends CommandBase {

    private final Swerve s_Swerve;
    private final Turret s_Turret;
    private final Shooter s_Shooter;
    private final FtcDashboard dashboard;

    private final GamepadEx m_Operator;

    TelemetryPacket packet = new TelemetryPacket();

    public JoystickTurret(Swerve s_Swerve, Turret s_Turret, Shooter s_Shooter, GamepadEx m_Operator, FtcDashboard dashboard){

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;
        this.s_Shooter = s_Shooter;
        this.dashboard = dashboard;

        this.m_Operator = m_Operator;

        addRequirements(s_Turret, s_Shooter);
    }

    @Override
    public void initialize(){

        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Shooter.setShooterAngle(0);

    }

    @Override
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

        s_Shooter.setShooterSpeed(m_Operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            s_Shooter.setShooterAngle(Constants.ShooterConstants.aimerUp);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            s_Shooter.setShooterAngle(Constants.ShooterConstants.aimerDown);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.START)) {
            s_Turret.resetTurretPosition();
        }

        m_Operator.readButtons();

        s_Turret.runToSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
