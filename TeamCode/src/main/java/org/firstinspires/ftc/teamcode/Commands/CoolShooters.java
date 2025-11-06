package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class CoolShooters extends CommandBase {

    private final EZTelemetry telem;

    private final Shooter s_Shooter;
    private final AprilVisionOnTurret s_Vision;

    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean shootersGunnaShoot;

    private double shooterPercent;

    public CoolShooters(Shooter s_Shooter, AprilVisionOnTurret s_Vision, GamepadEx m_Driver, GamepadEx m_Operator, EZTelemetry telem){

        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;

        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        this.telem = telem;

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize(){

        shootersGunnaShoot = false;
        shooterPercent = 0;

    }

    @Override
    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            s_Shooter.incrementSpeedConstant();
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            s_Shooter.decrementSpeedConstant();
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            s_Shooter.setShooterAngle(Constants.ShooterConstants.closeAngle);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            s_Shooter.setShooterAngle(Constants.ShooterConstants.farAngle);
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            shootersGunnaShoot = !shootersGunnaShoot;
        }

        if(shootersGunnaShoot){
            if(s_Vision.hasGoalTag()){
                shooterPercent = s_Shooter.getShooterSpeedFromDistance(s_Vision.getGoalDistance());
            } else {
                shooterPercent = 0.51;
            }
        } else {

            shooterPercent = 0;
        }

        s_Shooter.setShooterSpeed(shooterPercent);

        telem.putTelemetry("Shooter Percentage", shooterPercent);
        telem.putTelemetry("Shooter Velocity", s_Shooter.getShooterVelocity());
        telem.putTelemetry("Shooter Constant", s_Shooter.getShooterConstant());

    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setShooterSpeed(0);
    }
}
