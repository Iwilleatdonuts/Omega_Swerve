package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class SmartIntake extends CommandBase {

    private final Intake s_Intake;
    private final Feeder s_Feeder;
    private final FtcDashboard dashboard;

    private final GamepadEx m_Driver;

    TelemetryPacket packet = new TelemetryPacket();

    public SmartIntake(Intake s_Intake, Feeder s_Feeder, GamepadEx m_Driver, FtcDashboard dashboard){

        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;
        this.dashboard = dashboard;

        this.m_Driver = m_Driver;

        addRequirements(s_Intake, s_Feeder);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        s_Intake.setSpeed(m_Driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_Driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if(m_Driver.isDown(GamepadKeys.Button.A)) {
            s_Feeder.setFeederSpeed(1);
        } else if (m_Driver.isDown(GamepadKeys.Button.B)) {
            s_Feeder.setFeederSpeed(-1);
        } else {
            s_Feeder.setFeederSpeed(0);
        }

        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
