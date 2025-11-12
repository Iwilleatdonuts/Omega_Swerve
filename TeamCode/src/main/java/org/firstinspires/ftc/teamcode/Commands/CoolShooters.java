package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class CoolShooters {

    private final EZTelemetry telem;

    private final Shooter s_Shooter;
    private final AprilVisionOnTurret s_Vision;

    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean shootersGunnaShoot;

    private double shooterPercent;

    private double shooterAngle;

    public CoolShooters(Shooter s_Shooter, AprilVisionOnTurret s_Vision, GamepadEx m_Driver, GamepadEx m_Operator, EZTelemetry telem){

        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;

        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        this.telem = telem;
    }

    public void initialize(){

        shootersGunnaShoot = false;
        shooterPercent = 0;
        shooterAngle = 1;

    }

    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            s_Shooter.incrementSpeedConstant();
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            s_Shooter.decrementSpeedConstant();
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            shooterAngle = Constants.ShooterConstants.closeAngle;
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooterAngle = Constants.ShooterConstants.farAngle;
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            shootersGunnaShoot = !shootersGunnaShoot;
        }

        if(shootersGunnaShoot){
            if(s_Vision.hasGoalTag()){
                double distance = s_Vision.getGoalDistance();
                shooterPercent = s_Shooter.getShooterSpeedFromDistance(distance);
                shooterAngle = s_Shooter.getShooterAngleFromDistance(distance);
            } else {
                shooterPercent = 0.61;
                shooterAngle = Constants.ShooterConstants.farAngle;
            }
        } else {
            shooterPercent = 0;
        }

        s_Shooter.setShooterAngle(shooterAngle);
        s_Shooter.setShooterSpeed(shooterPercent);

        telem.putTelemetry("Shooter Percentage", shooterPercent);
        telem.putTelemetry("Shooter Velocity", s_Shooter.getShooterVelocity());
        telem.putTelemetry("Shooter Constant", s_Shooter.getShooterConstant());

    }

    public void end(boolean interrupted) {
        s_Shooter.setShooterSpeed(0);
    }
}
