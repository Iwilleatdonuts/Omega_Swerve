package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class CoolShooters extends CommandBase {

    private final Shooter s_Shooter;
    private final AprilVision s_Vision;

    private final GamepadEx m_Driver;

    private boolean shootersGunnaShoot;

    private double shooterPercent;

    private final Telemetry telemetry;

    public CoolShooters(Shooter s_Shooter, AprilVision s_Vision, GamepadEx m_Driver, Telemetry telemetry){

        this.s_Shooter = s_Shooter;
        this.s_Vision = s_Vision;

        this.m_Driver = m_Driver;

        this.telemetry = telemetry;

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize(){

        shootersGunnaShoot = false;
        shooterPercent = 0;

    }

    @Override
    public void execute(){

        if(m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            shootersGunnaShoot = !shootersGunnaShoot;
        }

        if(shootersGunnaShoot){
            if(s_Vision.hasGoalTag()){
                shooterPercent = s_Shooter.getShooterSpeedFromDistance(s_Vision.getGoalDistance());
            } else {
                shooterPercent = 0.7;
            }
        } else {

            shooterPercent = 0;
        }

        s_Shooter.setShooterSpeed(shooterPercent);

        telemetry.addData("Shooter Percentage", shooterPercent);
        telemetry.addData("Shooter Gunna shoob", shootersGunnaShoot);
        telemetry.update();

    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setShooterSpeed(0);
    }
}
