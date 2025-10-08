package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

public class TeleOpDrive extends CommandBase {

    private final Swerve s_Swerve;
    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean slowMode;
    private boolean dashboardDriving;

    public TeleOpDrive(Swerve s_Swerve, GamepadEx m_Driver, GamepadEx m_Operator){

        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;
        dashboardDriving = false;

        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            dashboardDriving = !dashboardDriving;
        }

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();

        if(dashboardDriving){
            xVal = m_Driver.getLeftY();
            yVal = m_Driver.getLeftX();
        }

        slowMode = m_Driver.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) || m_Driver.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        s_Swerve.drive(xVal, yVal, m_Driver.getRightX(), true, slowMode);

    }

}
