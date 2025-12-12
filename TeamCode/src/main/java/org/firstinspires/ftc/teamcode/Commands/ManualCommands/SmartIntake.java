package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class SmartIntake {

    public final EZTelemetry telem;

    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final Shooter s_Shooter;
    private final Turret s_Turret;

    private final Limelight s_Lime;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;

    private double seeTagTimestamp;
    private double loseTagTimestamp;
    private boolean allowedToShoot;

    public SmartIntake(Intake s_Intake, Feeder s_Feeder, Shooter s_Shooter, Turret s_Turret, Limelight s_Lime, OmegaController m_Driver, OmegaController m_Operator, EZTelemetry telem){

        this.telem = telem;
        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.s_Shooter = s_Shooter;
        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;

        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;
    }

    public void initialize(){
//        s_Intake.toggleTelemetry();
    }

    public void execute(){

        if(s_Intake.hasThreeBalls()&& m_Driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            m_Driver.setRumble(300);
        }

        s_Intake.setSpeed(m_Driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_Driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if (m_Operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9) {
            s_Feeder.openGate();
            s_Feeder.setFeederSpeed(1);
            s_Intake.setSpeed(1);
            allowedToShoot = false;
        } else if (m_Driver.isDown(GamepadKeys.Button.A)) {

            if(s_Shooter.shooterAtRoughSpeed() && s_Turret.atRoughSetpoint() && s_Shooter.getShooterVelocity() > 100) {
                s_Feeder.openGate();
                s_Feeder.setFeederSpeed(1);
                s_Intake.setSpeed(1);
            } else {
                s_Feeder.closeGate();
                s_Feeder.setFeederSpeed(0);
                s_Intake.setSpeed(0);
            }

        } else if (m_Driver.isDown(GamepadKeys.Button.B) || m_Driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0) {
            s_Feeder.setFeederSpeed(-1);
            s_Feeder.openGate();
            allowedToShoot = false;
        } else {
            s_Feeder.setFeederSpeed(0);
            s_Feeder.closeGate();
            allowedToShoot = false;
        }

        s_Intake.skadoodle();

    }
}
