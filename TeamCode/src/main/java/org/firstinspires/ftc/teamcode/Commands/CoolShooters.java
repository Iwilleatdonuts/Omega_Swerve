package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class CoolShooters {

    private final EZTelemetry telem;

    private final Shooter s_Shooter;
    private final Limelight s_Lime;
    private final FusionOdometry s_Lemon;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;

    private boolean shootersGunnaShoot;

    private double shooterPercent;

    private double shooterAngle;

    private final OmegaPose2D targetPose;
    private double timestamp;

    public CoolShooters(Shooter s_Shooter, Limelight s_Lime, FusionOdometry s_Lemon, OmegaController m_Driver, OmegaController m_Operator, EZTelemetry telem, boolean areWeWinners){

        this.s_Shooter = s_Shooter;
        this.s_Lime = s_Lime;
        this.s_Lemon = s_Lemon;

        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        this.targetPose = areWeWinners ? Constants.TurretConstants.redTarget : Constants.TurretConstants.blueTarget;

        this.telem = telem;
    }

    public void initialize(){

        shootersGunnaShoot = false;
        shooterPercent = 0;
        shooterAngle = 1;
        timestamp = System.nanoTime();

    }

    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            s_Shooter.incrementSpeedConstant();
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            s_Shooter.decrementSpeedConstant();
        }

        if(m_Operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            shooterAngle = Constants.ShooterConstants.farAngle;
        }

        if(m_Operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            shooterAngle = Constants.ShooterConstants.closeAngle;
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            shootersGunnaShoot = true;
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            shootersGunnaShoot = false;
        }

        if(shootersGunnaShoot){
                //MANUAL CLOSE SHOT
            if (m_Operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                shooterPercent = 0.38 + s_Shooter.getShooterConstant();
                shooterAngle = Constants.ShooterConstants.closeAngle;
                //MANUAL MEDOIUM SHOT
            } else if(m_Operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                shooterPercent = 0.45 + s_Shooter.getShooterConstant();
                shooterAngle = Constants.ShooterConstants.closeAngle-0.5;
                //MANUAL FAR SHOT
            } else if(m_Operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9) {
                shooterPercent = 0.56 + s_Shooter.getShooterConstant();
                shooterAngle = Constants.ShooterConstants.farAngle;
            } else if(s_Lime.isValidReaing()){
                double distance = s_Lime.getFilteredDistance();
                shooterPercent = s_Shooter.getShooterSpeedFromDistance(distance);
                shooterAngle = s_Shooter.getShooterAngleFromDistance(distance);
                timestamp = System.nanoTime();
            } else if(System.nanoTime() - timestamp > 1e9){
                double distance = Math.hypot(s_Lemon.getCurrentPose().x() - targetPose.x(), s_Lemon.getCurrentPose().y() - targetPose.y());
                shooterPercent = s_Shooter.getShooterSpeedFromDistance(distance);
                shooterAngle = s_Shooter.getShooterAngleFromDistance(distance);
            }
        } else {
            shooterPercent = 0;
        }

        s_Shooter.setShooterAngle(shooterAngle);
        s_Shooter.setShooterSpeed(shooterPercent);

        telem.putLine("Shooter");
        telem.putTelemetry("Shooter Percentage", shooterPercent);
        telem.putTelemetry("Target Velocity", s_Shooter.getTargetVelocity());
        telem.putTelemetry("Current Velocity", s_Shooter.getShooterVelocity());
        telem.putTelemetry("Shooter Constant", s_Shooter.getShooterConstant());
        telem.putDashboard("Shooter Percentage", shooterPercent);
        telem.putDashboard("Target Velocity", s_Shooter.getTargetVelocity());
        telem.putDashboard("Current Velocity", s_Shooter.getShooterVelocity());
        telem.putDashboard("Shooter Constant", s_Shooter.getShooterConstant());
        telem.putLine();
        telem.updateDashboard();

    }

    public void end(boolean interrupted) {
        s_Shooter.setShooterSpeed(0);
    }
}
