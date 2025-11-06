package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

public class SmartIntake extends CommandBase {

    private final Intake s_Intake;
    private final Feeder s_Feeder;

    private final GamepadEx m_Driver;

    double timestamp;

    public SmartIntake(Intake s_Intake, Feeder s_Feeder, GamepadEx m_Driver){

        this.s_Intake = s_Intake;
        this.s_Feeder = s_Feeder;

        this.m_Driver = m_Driver;

        addRequirements(s_Intake, s_Feeder);
    }

    @Override
    public void execute(){

        s_Intake.setSpeed(m_Driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_Driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if(m_Driver.isDown(GamepadKeys.Button.A)) {
            s_Feeder.openGate();
            s_Feeder.setFeederSpeed(1);
            s_Intake.setSpeed(1);
        } else if (m_Driver.isDown(GamepadKeys.Button.B)) {
            s_Feeder.setFeederSpeed(-1);
            s_Feeder.openGate();
        } else {
            s_Feeder.setFeederSpeed(0);
            s_Feeder.closeGate();
        }

    }
}
