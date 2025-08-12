package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Command Swerve")
public class CommandSwerveOpMode extends CommandOpMode {

    private Swerve s_Swerve;
    private OTOSSensor s_Sparky;
    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private FtcDashboard dashboard;

    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        s_Swerve = new Swerve(hardwareMap, telemetry);
        s_Sparky = new OTOSSensor(hardwareMap, telemetry);

        s_Swerve.setDefaultCommand(new Drive(s_Swerve, m_Driver, m_Operator));

        new GamepadButton(m_Driver, GamepadKeys.Button.START).whenPressed(new InstantCommand(() -> s_Sparky.zeroGyro()));

    }

}
